import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import re


def colourGradient(fromRGB, toRGB, steps=50):

    # So we can check format of input html-style colour codes
    hexRgbRe = re.compile('#?[0-9a-fA-F]{6}')
    # The code will handle upper and lower case hex characters, with or without a # at the front
    if not hexRgbRe.match(fromRGB) or not hexRgbRe.match(toRGB):
        # One of the inputs isn’t a valid rgb hex code
        raise Exception('Invalid parameter format')

    # Tidy up the parameters
    rgbFrom = fromRGB.split('#')[-1]
    rgbTo = toRGB.split('#')[-1]

    # Extract the three RGB fields as integers from each (from and to) parameter
    rFrom, gFrom, bFrom = [(int(rgbFrom[n:n+2], 16))
                           for n in range(0, len(rgbFrom), 2)]
    rTo, gTo, bTo = [(int(rgbTo[n:n+2], 16)) for n in range(0, len(rgbTo), 2)]

    # For each colour component, generate the intermediate steps
    rSteps = ['#{0:02x}'.format(
        round(rFrom + n * (rTo - rFrom) / (steps - 1))) for n in range(steps)]
    gSteps = ['{0:02x}'.format(
        round(gFrom + n * (gTo - gFrom) / (steps - 1))) for n in range(steps)]
    bSteps = ['{0:02x}'.format(
        round(bFrom + n * (bTo - bFrom) / (steps - 1))) for n in range(steps)]

    # Reassemble the components into a list of html-style #rrggbb codes
    return [r+g+b for r, g, b in zip(rSteps, gSteps, bSteps)]


def plot_3d(datas, times):
    wcs = np.array(datas)
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection='3d')

    lt = len(times)

    color_list = colourGradient('#ff0000', '#0000ff', lt)

    for i in range(lt):
        d = wcs[i, :, :]
        x = d[:, 0]
        y = d[:, 1]
        z = d[:, 2]
        ax.plot(x, y, z, marker='o', color=color_list[i])

    plt.show()
