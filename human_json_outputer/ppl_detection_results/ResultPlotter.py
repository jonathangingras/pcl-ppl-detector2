import numpy as np
import matplotlib.pyplot as plt

class ResultPlotter(object):
	def set_numpy_data_set(self, fileName, x_values, y_values):
		self.x = np.array(x_values)
		self.y = np.array(y_values)
		self.fig = plt.figure(fileName)

	def x_width(self):
		width = max(-(self.x.min()) - 0.5, self.x.max() + 0.5)
		yield -width, width
	
	def plot(self, xlimits = None, ylimits = None):
		if not xlimits:
			xlimits, = self.x_width()
		plt.xlim(xlimits)
		if ylimits:
			plt.ylim(ylimits)
		
		plot = self.fig.add_subplot(111)

		plot.scatter(self.x, self.y, alpha=0.5)
		
		plot.set_xlabel('gauche(negatif) to droite(positif) (m)')
		plot.set_ylabel('Profondeur (m)')

		plot.set_xlim([-5,5])
		plot.set_ylim([0,10])
		
		plt.show()


class LegTrackerResultPlotter(ResultPlotter):
	def __init__(self, fileName, loadedResults, threshold = None):
		x_values = []
		y_values = []

		for sample in loadedResults:
			for leg in sample["legs"]:
				#if (not threshold) or (threshold and leg["probability"] >= threshold):
				x_values.append(leg["x"]) # the x on the plane corresponds to LegTracker -Y axis
				y_values.append(leg["y"])  # the y on the plane corresponds to LegTracker X axis (depth)

		self.set_numpy_data_set(fileName, x_values, y_values)
		#plt.title('People Tracker Results')
