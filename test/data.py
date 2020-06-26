import numpy as np
import numpy.random as npr
import matplotlib.pyplot as plt

def clip_normal():
    ans = npr.normal(0, 5)
    while np.abs(ans) > 15:
        ans = npr.normal(0, 5)
    return ans

theta = 0
ans = []

while len(ans) < 3000:
    v = npr.uniform(20, 30)
    
    new_theta = theta + v / np.pi
    pos0 = 200 + 100 * np.cos(new_theta)
    pos1 = 200 + 100 * np.sin(new_theta)
    theta = np.mod(new_theta, np.pi * 2)

    ans.append([pos0, pos1])

x = []
y = []
for each in ans:
    x.append(each[0])
    y.append(each[1])

plt.scatter(x, y, s=1)
plt.show()

print(x)

with open('circle.truth.txt', 'w') as f:
    for each in ans:
        f.write(f'{each[0]} {each[1]}\n')

with open('circle.noise.txt', 'w') as f:
    for each in ans:
        f.write(f'{each[0] + clip_normal()} {each[1] + clip_normal()}\n')
