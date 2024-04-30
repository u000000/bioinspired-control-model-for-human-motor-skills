# bioinspired-control-model-for-human-motor-skills

Creating a hand writing simulation.<br /> 
1. User signs on the tablet.<br />
2. Picture is uploaded to the Python OpenCV script and converted to .csv file which containing points of the path.<br />
3. Controller function performs calculations and joint angles data is saved as .csv file.
4. .csv is sourced in CoppeliaSim.<br />
5. Using python script in CoppeliaSim simulator iterate through the .csv file and moves a robot arm accordingly.<br />
