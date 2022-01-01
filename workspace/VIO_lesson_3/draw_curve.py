import matplotlib.pyplot as plt
import numpy as np

def parse_txt(file: str):
  iters_index = []
  chi_value = []
  lambda_value = []
  
  with open(file, 'r') as f:
    for line in f:      
      ind_iter = line.find("iter: ")
      ind_chi = line.find("chi=")
      ind_lambda = line.find("Lambda=")

      iters_index.append(float(line[ind_iter+6: ind_chi-3]))
      chi_value.append(float(line[ind_chi+5 : ind_lambda-3]))
      lambda_value.append(float(line[ind_lambda+8:-1]))
      
    for iter, chi, lamd in zip(iters_index, chi_value, lambda_value):
      print(f"iter: {iter}, chi: {chi}, lambda: {lamd}")
  
  return iters_index, chi_value, lambda_value


def draw_curve(xs, ys, zs):
  xs_array = np.array(xs, dtype=np.float32)
  ys_array = np.array(ys, dtype=np.float32)
  zs_array = np.array(zs, dtype=np.float32)
  
  plt.plot(xs_array, ys_array, color="blue")
  plt.scatter(xs_array, ys_array, color="blue")
  
  # plt.plot(xs_array, zs_array, color="red")
  # plt.scatter(xs_array, zs_array, color="red")
      
  # plt.xlabel("iterations")
  # plt.ylabel("lambda / chi2")
  plt.show()
  

if __name__ == "__main__":
  iters, chi2s, lambdas = parse_txt("docs/task1_3_result.txt")
  draw_curve(iters, lambdas, chi2s)
  
  