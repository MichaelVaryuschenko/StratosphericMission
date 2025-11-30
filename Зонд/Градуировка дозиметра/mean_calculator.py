import numpy as np;
from matplotlib import pyplot as plt;
file_names = ["Ambient", "Am-241", "Co-60", "Cs-137", "Eu-152", "Na-22"];
print("Average dosimeter readings");
step = 12;
x = np.arange(0, 300, step);
ind = 1;
for name in file_names:
    file = open(name + ".txt");
    data = np.array([int(row.split()[2][:len(row.split()[2]) - 1]) for row in file.readlines()]);
    avg = np.mean(data);
    err = np.std(data);
    data_rounded = data // step * step;
    bars = [len(data_rounded[data_rounded == elem]) / len(data_rounded) for elem in x];
    plt.subplot(2, int(np.ceil(len(file_names) / 2)), ind);
    plt.title(name);
    plt.bar(x / 62, bars, width=step / 62);
    print(name + ":", avg, "error:", err);
    ind += 1;
    plt.minorticks_on();  # ЭТО РИСОЧКИ НА САМИХ ОСЯХ!!!!!!!!!!!
    plt.grid(which='major', linewidth=0.2);
    plt.grid(which='minor', linewidth=0.2);
    plt.xlabel("Доза излучения, мкЗв/ч");
    plt.ylabel("Вероятность");

plt.tight_layout();
plt.show();
