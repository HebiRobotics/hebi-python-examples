import flask
from flask import request, render_template
from HebiThread import HebiThread



app = flask.Flask(__name__)
app.config["DEBUG"] = False


@app.route('/', methods=['GET'])
def home():
  return '''<h1>Distant Reading Archive</h1>
<p>A prototype API for distant reading of science fiction novels.</p>'''

@app.route('/index', methods=['GET', 'POST'])
def index():
  # request.
  # print("waypoint")
  # if request.args == "waypoint_target":
  #     print("waypoint")
  # print(request.data)
  # return render_template('index.html')

  if request.method == "POST":
    print(request.form['waypoint_target'])

    if request.form['waypoint_target'] == 'Waypoint 1':
      hebi_thread.set_waypoint_1()

    if request.form['waypoint_target'] == 'Waypoint 2':
      hebi_thread.clear_waypoints()
    # print(request.form['waypoint2'])


  return render_template('index.html')
     


hebi_thread = HebiThread()
app.run()

# #app.py

# from flask import Flask, request #import main Flask class and request object

# app = Flask(__name__) #create the Flask app

# @app.route('/query-example')
# def query_example():
#     return 'Todo...'

# @app.route('/form-example')
# def formexample():
#     return 'Todo...'

# @app.route('/json-example')
# def jsonexample():
#     return 'Todo...'

# app.run()
