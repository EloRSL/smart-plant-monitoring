"""Plant photo analysis: the two pipelines from the class labs, run on a camera photo.

leafs-starter  : U2-Net marks the foreground (leaf) pixels, then hand-written HSV colour
                 rules measure how green / yellow / brown those pixels are.
                 Works on any plant, but U2-Net is a heavy network: the EXPENSIVE pipeline.
leafs-advanced : a MobileNetV3 classifier scores 15 crop conditions, grouped here into
                 healthy / unhealthy. It only knows pepper, potato and tomato leaves,
                 but it is a small, fast network: the CHEAP pipeline (about 27x faster on the Pi).

The code follows the lab handouts step by step. The models and helpers.py are used straight
from the lab folders, so there is only one copy of each.
"""
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
from PIL import Image

PI = Path(__file__).resolve().parent
STARTER = PI.parent / "leafs-starter"
ADVANCED = PI.parent / "leafs-advanced"
PHOTO = PI / "data" / "last_photo.jpg"  # common.py creates data/

sys.path.insert(0, str(STARTER))
from helpers import color_fracs, read_and_resize, split_leaf_masks  # noqa: E402

# Fixed preprocessing settings of the leafs-advanced model (lab step 0).
RESIZE_SIZE = 256
CROP_SIZE = 224
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
TEMPERATURE = 1.210394024848938


def start_photo(size=None):
    """Start the Pi camera taking one photo into last_photo.jpg and return the process.

    It returns immediately, so the caller can do other work during the exposure.
    size=None is the camera's full resolution.
    """
    command = ["rpicam-still", "--nopreview", "--timeout", "1000", "--output", str(PHOTO)]
    if size:
        command += ["--width", str(size[0]), "--height", str(size[1])]
    return subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


def load_models(*pipelines):
    """Load the network of each requested pipeline once (lab step 1): "starter", "advanced".

    analyze_file() runs only what was loaded here, so this call decides how much work a photo
    costs. Measured on the Pi: starter about 2.2 s per photo, advanced about 0.08 s.
    """
    if not pipelines or set(pipelines) - {"starter", "advanced"}:
        raise ValueError(f'pipelines must be "starter" and/or "advanced", got {pipelines}')
    cpu = ["CPUExecutionProvider"]
    models = {}
    if "starter" in pipelines:
        models["u2net"] = ort.InferenceSession(str(STARTER / "u2netp.onnx"), providers=cpu)
    if "advanced" in pipelines:
        models["classifier"] = ort.InferenceSession(str(ADVANCED / "model.onnx"), providers=cpu)
        models["labels"] = json.loads((ADVANCED / "labels.json").read_text())
    return models


def leaf_masks(u2net, image):
    """leafs-starter checkpoints 2-4: saliency map -> clean binary mask -> one mask per region."""
    # Checkpoint 2: BGR -> RGB, 320 x 320, float32 in [0, 1], HWC -> NCHW.
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (320, 320), interpolation=cv2.INTER_AREA)
    pixels = resized.astype(np.float32) / 255.0
    batch = np.ascontiguousarray(np.expand_dims(pixels.transpose(2, 0, 1), axis=0))

    # Checkpoint 3: the first output is the saliency map, one score per pixel.
    saliency = u2net.run(None, {u2net.get_inputs()[0].name: batch})[0][0, 0]

    # Checkpoint 4: scale to 0-255, back to the photo size, Otsu threshold, close, open.
    low, high = saliency.min(), saliency.max()
    scaled = (255 * (saliency - low) / (high - low + 1e-8)).astype(np.uint8)
    height, width = image.shape[:2]
    full_size = cv2.resize(scaled, (width, height), interpolation=cv2.INTER_LINEAR)
    _, binary = cv2.threshold(full_size, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), dtype=np.uint8)
    clean = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    clean = cv2.morphologyEx(clean, cv2.MORPH_OPEN, kernel, iterations=1)
    return split_leaf_masks(clean)


def unhealthy_score(classifier, labels, image):
    """leafs-advanced steps 2-4: prepare the tensor, run the network, sum the disease classes."""
    # Step 2: RGB, shorter side to 256, centre crop 224, normalise, HWC -> NCHW.
    photo = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    width, height = photo.size
    long_side = int(RESIZE_SIZE * max(width, height) / min(width, height))
    size = (RESIZE_SIZE, long_side) if width <= height else (long_side, RESIZE_SIZE)
    photo = photo.resize(size, Image.Resampling.BILINEAR)
    left = round((photo.width - CROP_SIZE) / 2)
    top = round((photo.height - CROP_SIZE) / 2)
    photo = photo.crop((left, top, left + CROP_SIZE, top + CROP_SIZE))
    pixels = np.asarray(photo, dtype=np.float32) / np.float32(255.0)
    pixels = (pixels - MEAN) / STD
    batch = np.ascontiguousarray(np.expand_dims(pixels.transpose(2, 0, 1), axis=0))

    # Step 3: 15 raw scores (logits), one per class.
    logits = classifier.run(None, {classifier.get_inputs()[0].name: batch})[0][0]

    # Step 4: softmax, then add up every class that is not a "healthy" one.
    scores = logits.astype(np.float64) / TEMPERATURE
    weights = np.exp(scores - scores.max())  # subtracting the max avoids overflow in exp()
    probabilities = weights / weights.sum()
    healthy = np.array([name.lower().endswith("healthy") for name in labels])
    return float(probabilities[~healthy].sum())


def analyze_file(path, models):
    """Run the loaded pipelines on an image file. Returns values usable in configuration.txt.

    starter:  leaf_count        number of leaf-like regions (touching leaves count as one)
              green/yellow/brown_percent   share of the leaf pixels with that colour
    advanced: unhealthy_percent classifier's score for "diseased" (pepper/potato/tomato only)
    """
    image = read_and_resize(str(path))  # longest side capped at 1200, as in the lab
    result = {}
    if "u2net" in models:
        masks = leaf_masks(models["u2net"], image)
        result["leaf_count"] = len(masks)
        if masks:  # no region = no colour measurement; it does not mean the plant is unhealthy
            fractions = color_fracs(image, np.maximum.reduce(masks))
            result.update({f"{name}_percent": round(100 * value) for name, value in fractions.items()})
    if "classifier" in models:
        score = unhealthy_score(models["classifier"], models["labels"], image)
        result["unhealthy_percent"] = round(100 * score)
    return result


def wait_photo(photo):
    """Wait until a photo started with start_photo() is saved in last_photo.jpg."""
    _, errors = photo.communicate()
    if photo.returncode != 0:
        raise RuntimeError(f"camera failed: {errors.decode(errors='replace')}")
