
#!/usr/bin/python
# coding: utf-8

import matplotlib.pyplot as plt

filename = "../data/plot_Z_Accel_example.txt" 
x_data = []   
y_data = []   

with open(filename, "r") as file:
    for line in file:
        line = line.strip()  
        if line:
            data = line.split()   
            x_data.append(float(data[0]) - 5 )   
            y_data.append(float(data[1]))   
            print(float(data[0]), float(data[1]))

# 
# plt.scatter(x_data, y_data)
plt.plot(x_data, y_data, color='blue', linestyle='-', linewidth=1)


# plt.title("Scatter Plot")
plt.xlabel("Time [sec] " , fontsize=15)
plt.ylabel(" Z Accel [m/s/s] " , fontsize=15)
plt.grid()
plt.xlim(0 , 10)
plt.ylim(0, 20)
plt.tick_params(axis='both', which='major', labelsize=16)  
plt.locator_params(axis='y', nbins= 8 )

plt.show()
