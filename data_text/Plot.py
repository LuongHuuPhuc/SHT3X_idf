import pandas as pd
import matplotlib.pyplot as plt

# Đọc file CSV
df = pd.read_csv(r"D:\Esp-idf\SHT3x_driver\data_text\2h36.csv")

# Vẽ biểu đồ
plt.figure(figsize=(12, 6))
plt.plot(df['TempC'], label='Temperature (°C)', color='red', marker='o')
plt.plot(df['TempF'], label='Temperature (°F)', color='orange', linestyle='--', marker='x')
plt.plot(df['Humi'], label='Humidity (%)', color='green', marker='s')

# Thêm tiêu đề và nhãn
plt.title('SHT3x Sensor Data')
plt.xlabel('Sample Index')
plt.ylabel('Value')
plt.legend()
plt.grid(True)

# Hiển thị biểu đồ
plt.tight_layout()
plt.show()
