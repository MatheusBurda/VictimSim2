import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.metrics import mean_squared_error, r2_score
import joblib
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
import joblib

def train_model(train_data_filename):
    print(f'Training model {train_data_filename}')

    train_data = pd.read_csv(train_data_filename)
    df_train = pd.DataFrame(train_data)

    x_train = df_train[['qPA', 'pulso', 'freqResp']]
    y_train = df_train['gravidade']

    model = Sequential()

    model.add(Dense(64, input_dim=x_train.shape[1], activation='relu'))
    model.add(Dense(32, activation='relu'))
    model.add(Dense(1, activation='linear'))

    model.compile(optimizer=Adam(learning_rate=0.001), loss='mean_squared_error', metrics='mean_squared_error')

    model.fit(x_train, y_train, epochs=100, batch_size=32, verbose=1)

    return model

def test_model(model, test_data_filename):
    print(f'Testing model using data from {test_data_filename}')

    test_data = pd.read_csv(test_data_filename)
    df_test = pd.DataFrame(test_data)

    x_test = df_test[['qPA', 'pulso', 'freqResp']]
    y_test = df_test['gravidade']

    print(f'Predicting...')
    y_pred = model.predict(x_test)

    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    print(f'Mean Squared Error: {mse}')
    print(f'R^2 Score: {r2}')

    return y_pred


def predict(model, data: pd.DataFrame):

    y_pred = model.predict(data)

    return list(y_pred)


def save_model(model, save_model_filename):
    joblib.dump(model, save_model_filename)


def load_model(filename):
    model = joblib.load(filename)

    return model


if __name__ == "__main__":
    model = train_model(train_data_filename='datasets/data_4000v/env_vital_signals.txt')

    save_model(model, save_model_filename='neural_network_model.pkl')

    model = load_model('neural_network_model.pkl')

    test_model(model, 'datasets/data_800v/env_vital_signals.txt')
    