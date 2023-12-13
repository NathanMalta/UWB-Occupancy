import csv
import time

timestamps = []
passengers = []

while True:
    x = input("How many people are on the bus?: ")
    if x == "close":
        with open("ground_truth.csv", mode='a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            for item1, item2 in zip(timestamps, passengers):
                writer.writerow([item1, item2])
            print("Successfully saved!")
        break
    elif x.isnumeric():
        passengers.append(int(x))
        timestamps.append(time.time())