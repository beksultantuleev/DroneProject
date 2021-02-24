import numpy as np 
import matplotlib.pyplot as plt 
  
in_array = np.linspace(-np.pi, np.pi, 12) 
out_array = np.sin(in_array) 
  
print("in_array : ", in_array) 
print("\nout_array : ", out_array) 
  
# red for numpy.sin() 
plt.plot(in_array, out_array, color = 'red', marker = "o") 
plt.title("numpy.sin()") 
plt.xlabel("X") 
plt.ylabel("Y") 
plt.show() 
