import numpy as np
import matplotlib.pyplot as plt

def main():
    et = np.loadtxt('output/error-t-table.csv', delimiter=',').T
    t, sx, angle, h = et[0], et[1], et[2], et[3]
    plt.plot(t, angle, t, sx, t, h)
    plt.legend(['angle', 'sx', 'h'])
    # plt.plot(t, angle)
    plt.show()
    plt.savefig('error-time.jpg')

if __name__ == '__main__':
    main()