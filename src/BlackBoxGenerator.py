import logging
from datetime import date
import json
import ast
from collections import defaultdict
import time
import os
import threading


class Logger:
    def __init__(self):
        today = date.today()
        today = today.strftime("%b-%d-%Y")
        clock = time.ctime()[11:-5].replace(':', "")
        self.filename = "logs/raw/raw_" + today + "-" + clock + ".json"
        self.final_log_name = "logs/" + today + "-" + clock + ".json"

    # def start(self, *args):
    #     thread = threading.Thread(target=self.start_logging)
    #     thread.start()

    def start_logging(self, *args):
        logging.basicConfig(filename=self.filename,
                            format='{%(message)s}', level=logging.DEBUG)
        for data in args:
            logging.info(f'"{data[0]}":{data[1]}')

        dd = defaultdict(list)

        with open(self.filename) as file:
            data = file.readlines()
            for i in data:
                for key, value in ast.literal_eval(i).items():
                    dd[key].append(value)

        with open(self.final_log_name, "w") as writefile:
            json.dump(dd, writefile)


if __name__ == "__main__":
    test = Logger()
    thread = threading.Thread(target=test.start_logging(["IMU", [4, 3, 5]], ["UWB", [5353, 464, 456]], [
        "IMU", [234, 345, 53]], ["UWB", [4, 2, 4]],  [
        "IMU", [234, 345, 53]]))
    thread.start()
