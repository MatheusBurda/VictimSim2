import pandas as pd
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.ensemble import GradientBoostingRegressor
import joblib


def train_model(train_data_filename):

    print(f'Training model {train_data_filename}')
    # Loads the data
    train_data = pd.read_csv(train_data_filename)
    df_train = pd.DataFrame(train_data)

    # selecting only relevant columns (features e output)
    x_train = df_train[['qPA', 'pulso', 'freqResp']]
    y_train = df_train['gravidade']
    model = GradientBoostingRegressor(loss='absolute_error', max_depth=100, random_state=32)
    
    # Treinando o modelo
    model.fit(x_train, y_train)

    print(f'Model successfully trained')

    return model


def test_model(model, test_data_filename):
    
    print(f'Testing model using data from {test_data_filename}')

    test_data = pd.read_csv(test_data_filename)
    df_test = pd.DataFrame(test_data)

    x_test = df_test[['qPA', 'pulso', 'freqResp']]
    y_test = df_test['gravidade']

    print('Predicting...')
    y_pred = model.predict(x_test)

    # Avaliando o modelo
    mse = mean_squared_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    print(f'Results:\nMSE: {mse}\nr2: {r2}')


def save_model(model, save_model_filename='gradient_boosting_model.pkl'):
    joblib.dump(model, save_model_filename)


def load_model(filename):
    model = joblib.load(filename)

    return model



if __name__ == "__main__":
    model = train_model(train_data_filename='datasets/data_4000v/env_vital_signals.txt')

    save_model(model, save_model_filename='gradient_boosting_model.pkl')
    
    model = load_model('gradient_boosting_model.pkl')

    test_model(model, 'datasets/data_800v/env_vital_signals.txt')

