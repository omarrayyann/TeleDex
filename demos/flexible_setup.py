from teledex import Session

session = Session()
session.start()

while True:
    data = session.get_latest_data()
    if data["position"] is not None:
        print(data)