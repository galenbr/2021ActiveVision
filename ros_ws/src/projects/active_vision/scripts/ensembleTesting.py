import numpy as np
from tensorflow import keras
from tensorflow.keras import layers
from keras.models import Model
import tensorflow as tf

def my_loss_fn(y_true, y_pred, y_var):
	return tf.keras.losses.categorical_crossentropy(y_true, y_pred)

# Model / data parameters
num_classes = 10
input_shape = (28, 28, 1)

# the data, split between train and test sets
(x_train, y_train), (x_test, y_test) = keras.datasets.mnist.load_data()

# Scale images to the [0, 1] range
x_train = x_train.astype("float32") / 255
x_test = x_test.astype("float32") / 255
# Make sure images have shape (28, 28, 1)
x_train = np.expand_dims(x_train, -1)
x_test = np.expand_dims(x_test, -1)
print("x_train shape:", x_train.shape)
print(x_train.shape[0], "train samples")
print(x_test.shape[0], "test samples")


# convert class vectors to binary class matrices
y_train = keras.utils.to_categorical(y_train, num_classes)
y_test = keras.utils.to_categorical(y_test, num_classes)

inputs = keras.Input(shape=input_shape)
conv1 = layers.Conv2D(32, kernel_size=(3, 3), activation="relu")(inputs)
pool1 = layers.MaxPooling2D(pool_size=(2, 2))(conv1)
conv2 = layers.Conv2D(64, kernel_size=(3, 3), activation="relu")(pool1)
pool2 = layers.MaxPooling2D(pool_size=(2, 2))(conv2)
flat = layers.Flatten()(pool2)
dropout = layers.Dropout(0.5)(flat)

class_output = layers.Dense(num_classes, activation="softmax")(dropout)
variance_output = layers.Dense(1, activation="softplus")(dropout)
epsilons = tf.convert_to_tensor([1e-6])
variance_output = layers.Add()([variance_output, epsilons])

model = Model(inputs, [class_output, variance_output])

model.summary()

batch_size = 128
epochs = 2

model.compile(loss=my_loss_fn, optimizer="adam", metrics=["accuracy"])

model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, validation_split=0.1)

score = model.evaluate(x_test, y_test, verbose=0)
print("Test loss:", score[0])
print("Test accuracy:", score[1])
