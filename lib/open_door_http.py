import requests

def open_door(ESP_IP:str):
    url = 'http://'+ESP_IP+'/openDoor'
    myobj = {"action": "openDoor", "parameters": {"robotId": "42"}}
    response = requests.post(url, json = myobj)
    print(response.json())
    if response.status_code == 400:
        print("HTTP Error")
        raise requests.HTTPError
    door = response.json()["data"]["message"]
    # print(door)
    return 1 if door == "door1" else (2 if door == "door2" else 0) # print 0 means unexpected response

def main(args=None):
    print(open_door("192.168.67.199"))

if __name__ == "__main__":
    main()
