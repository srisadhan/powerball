import random

fine  = ['HighFine','LowFine']
gross = ['HighGross','LowGross']
comb  = ['HighComb', 'LowComb', 'CombComb']
task  = [fine, gross]

random.shuffle(fine)
random.shuffle(gross)
random.shuffle(comb)

random.shuffle(task)
task = [task, comb]

print("Randomized task selection for the subject: {}".format(task))