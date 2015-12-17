import CleaningData as cd

def createModels():
	emotions = ["come", "stop", "goodBoy", "fetch"]

	#collect training data names from folders
	people = ['jamie', 'susie']
	direction = {'Close': 4, 'Far':4}

	extract = cd.DataIntake(emotions, people)
	extract.createModelDictionary()