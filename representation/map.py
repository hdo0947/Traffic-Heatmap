import numpy as np
import gmplot
import scipy.io as scio

intensity_points = scio.loadmat('./representation/intensity_points.mat')["intensity_points"]
# Create the map plotter:
apikey = 'AIzaSyDJSghdJBc-Peznow3d9aQyz4pycisnkjI'
center = np.mean(intensity_points[:, 0:2], axis=0)
gmap = gmplot.GoogleMapPlotter(center[0], center[1], 18  , apikey=apikey)
green = []
red = []
yellow = []
for point in intensity_points:
    if point[2] == 0:
        green.append((point[0], point[1]))
    elif point[2] == 1:
        yellow.append((point[0], point[1]))
    else:
        red.append((point[0], point[1]))

    if point[3] == 0:
        green.append((point[0], point[1]-0.0003))
    elif point[3] == 1:
        yellow.append((point[0], point[1]-0.0003))
    else:
        red.append((point[0], point[1]-0.0003))


red_attractions_lats, red_attractions_lngs = zip(*red)
yellow_attractions_lats, yellow_attractions_lngs = zip(*yellow)
green_attractions_lats, green_attractions_lngs = zip(*green)
gmap.scatter(red_attractions_lats, red_attractions_lngs, color='#ff0000', size=1, marker=False)
gmap.scatter(yellow_attractions_lats, yellow_attractions_lngs, color='#ffff00', size=1, marker=False)
gmap.scatter(green_attractions_lats, green_attractions_lngs, color='#00ff00', size=1, marker=False)

gmap.marker(intensity_points[0, 0], intensity_points[0, 1], color='cornflowerblue')
gmap.marker(intensity_points[-1, 0], intensity_points[-1, 1], color='red')

gmap.draw('map.html')
print("done")