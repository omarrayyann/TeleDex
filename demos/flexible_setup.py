from teledex import TeledexConnector

connector = TeledexConnector()
connector.start()

while True:
    data = connector.get_latest_data()
    if data["position"] is not None:
        print(data)