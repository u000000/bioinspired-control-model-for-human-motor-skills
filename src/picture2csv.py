import cv2
import pandas as pd
import numpy as np
import os

def picture2csv(file_name:str) -> pd.DataFrame:
    img = cv2.imread(file_name, cv2.IMREAD_GRAYSCALE)
    _, binary_image = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY)

    # Look for black pixels
    black_line_points = np.argwhere(binary_image == 0)

    df = pd.DataFrame(black_line_points)
    df.to_csv('line.csv', index=False, header=False)

    # Use imshow to show the line exported to csv
    cv2.imshow('path',binary_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return df


def controller(df_points:pd.DataFrame) -> None:
    # Generate random joint angles
    joints_angles = np.random.randint(10, 30, (df_points.shape[0], 2))

    df = pd.DataFrame(joints_angles, columns=['joint1','joint2'])
    df.to_csv('jointAngles.csv', index=False, header=False)

if __name__ == "__main__":
    path = os.path.dirname(__file__)
    file_name = os.path.join(path, 'signature.png')
    df_points = picture2csv(file_name)
    controller(df_points)