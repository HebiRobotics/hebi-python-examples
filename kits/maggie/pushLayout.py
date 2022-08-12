import requests

message = open("./kits/maggie/layouts/mainLayout.json", "r")

address = 'http://10.10.10.160'
res = requests.post(address, message)
print(res)