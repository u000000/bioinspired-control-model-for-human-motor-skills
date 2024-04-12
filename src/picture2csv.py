# Write a function which will import a picture with the black line on the white background. Save black line points to the .csv file (as a path, so the closest one has to be saved one after another). The function should return a pandas DataFrame with the points. Line in the picture is a one pixel line.

import cv2
import pandas as pd
import numpy as np

def picture2csv(file_name:str) -> pd.DataFrame:
    img = cv2.imread(file_name, cv2.IMREAD_GRAYSCALE)
    _, binary_image = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY)

    # Look for black pixels
    black_line_points = np.argwhere(binary_image == 0)

    # Add third column with zeros
    black_line_points = np.insert(black_line_points, 2, 0, axis=1)

    df = pd.DataFrame(black_line_points, columns=['x','y','z'])
    df.to_csv('line.csv', index=False)

    # Use imshow to show the line exported to csv
    cv2.imshow('path',binary_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return df


def controller(df_points:pd.DataFrame) -> None:
    # Generate random joint angles
    joints_angles = np.random.randint(10, 80, (df_points.shape[0], 2))

    df = pd.DataFrame(joints_angles, columns=['joint1','joint2'])

    df.to_csv('joint_angles.csv', index=False)

if __name__ == "__main__":
    file_name = 'signature.png'
    df_points = picture2csv(file_name)
    controller(df_points)