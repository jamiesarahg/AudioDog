import CleaningData as cd

def createModels():
	emotions = ["come", "stop", "goodBoy", "fetch"]

	#collect training data names from folders
	people = ['jamie', 'susie']
	direction = {'Close': 4, 'Far':4}

	extract = cd.DataIntake(emotions, people)
	for person in extract.names:
		extract.collectTrainingData(person)
	result_dict = extract.createModelDictionary()
	print result_dict
	return result_dict


if __name__ == '__main__':
	createModels()