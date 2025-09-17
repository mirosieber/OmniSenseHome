import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

# Gegebene Punkte
x_data = np.array([510, 1, 0.1, 25])
y_data = np.array([255, 10, 0, 100])

# Funktion mit Parametern a, b, c, n
def model(params, x):
    a, b, c, n = params
    # Damit keine komplexen Zahlen entstehen, stellen wir sicher, dass x - b >= 0
    val = x - b
    val[val < 0] = 0  # klammere negative Werte aus, damit sqrt definiert ist
    return a * np.power(val, 1/n) + c

# Residuenfunktion für least_squares (Differenz zwischen Modell und Daten)
def residuals(params, x, y):
    return model(params, x) - y

# Startwerte (z.B. a=1, b=0, c=0, n=2)
start_params = [1.0, 0.0, 0.0, 2.0]

# Parameterbeschränkungen:
# - n > 0 (Wurzelexponent positiv)
# - x - b >= 0 für alle x in Daten → b <= min(x)
bounds_lower = [-np.inf, -np.inf, -np.inf, 0.0001]
bounds_upper = [np.inf, np.min(x_data), np.inf, np.inf]

# Optimierung
result = least_squares(residuals, start_params, args=(x_data, y_data), bounds=(bounds_lower, bounds_upper))

a_opt, b_opt, c_opt, n_opt = result.x

print("f(x)=a⋅(x−b)^(1/n)+c")

print(f"Optimierte Parameter:")
print(f"a = {a_opt}")
print(f"b = {b_opt}")
print(f"c = {c_opt}")
print(f"n = {n_opt}")

# Test: Werte für x_data
y_pred = model(result.x, x_data)
print("\nVergleich Original vs. Modell:")
for xi, yi, ypi in zip(x_data, y_data, y_pred):
    print(f"x={xi}: Original y={yi}, Modell y={ypi}")


# Wertebereich für den Plot
x_plot = np.linspace(np.min(x_data), np.max(x_data), 200)
y_plot = model(result.x, x_plot)

plt.figure(figsize=(8, 5))
plt.plot(x_plot, y_plot, label='Modellfunktion', color='blue')
plt.scatter(x_data, y_data, color='red', label='Originaldaten')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Angepasste Wurzelfunktion')
plt.legend()
plt.grid(True)
plt.show()
