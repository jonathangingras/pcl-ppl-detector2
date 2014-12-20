import json
from ppl_detection_results.ResultLoader import *

class JSONResultLoader(ResultLoader):
	def __init__(self, pfileName):
		super(JSONResultLoader, self).__init__(pfileName)
	def loadSamples(self):
		with open(self.fileName, "r") as res_file:
			return json.loads(res_file.read().replace('\n', ''))["samples"]