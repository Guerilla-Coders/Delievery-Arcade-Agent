import requests
import json


def fetch_config(target: str) -> dict:
    response = requests.get(f"http://49.50.175.88:6000/config/{target}")
    fetched = json.loads(response.text)
    return fetched
