import logging
from datetime import date
import json
import ast
from collections import defaultdict


class Logger:
    def __init__(self):
        today = date.today()
        today = today.strftime("%b-%d-%Y")
        self.filename = "logs/raw_log_" + today + ".json"
        self.final_log_name = "logs/"+today + ".json"

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
    test.start_logging(["imu_data", [4, 3, 5]], ["UWB_data", [5353, 464, 456]], [
                       "imu_data", [234, 345, 53]], ["UWB_data", [4, 2, 4, 5]])
