from teledex import Session

connector = Session()
connector.start()

while True:
    data = connector.get_latest_data()
    if data["position"] is not None:
        print(data)