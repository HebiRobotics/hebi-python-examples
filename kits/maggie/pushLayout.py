import requests

message = open("./kits/maggie/layouts/Front.json", "r")

address = 'http://10.10.10.160'
res = requests.post(address, message)
print(res)