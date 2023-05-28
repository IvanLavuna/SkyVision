from keras.applications.vgg16 import VGG16
from keras.applications.vgg16 import preprocess_input
from keras.preprocessing import image
from sklearn.preprocessing import normalize
from scipy.spatial import distance
import numpy as np

# Load VGG16 model, include_top=False to load model without the fully-connected layers
model = VGG16(weights='imagenet', include_top=False)


def _get_features(img: np.ndarray):
    img = np.expand_dims(img, axis=0)
    img = preprocess_input(img)
    features = model.predict(img)
    return features.flatten()


def similarity_score(img1: np.ndarray, img2: np.ndarray) -> float:
    """
    :brief:  The smaller the distance, the more similar the images are considered to be.
    :return: similarity score
    """
    # Get features for two images
    img1_features = _get_features(img1)
    img2_features = _get_features(img2)

    # Normalize the feature vectors
    img1_features_norm = normalize([img1_features])
    img2_features_norm = normalize([img2_features])

    # Compute similarity (Euclidean distance) between the features
    similarity = distance.euclidean(img1_features_norm, img2_features_norm)
    return similarity
