import flask
from flask import request, render_template
from HebiThread import HebiThread

app = flask.Flask(__name__)
app.config["DEBUG"] = False
# Note: If the above is True, it will start 2 HebiThreads and cause issues

@app.route('/', methods=['GET'])
def home():
  return '''<h1>Go to /index for the main screen</h1>
<p>Go to /index for the main screen</p>'''

@app.route('/index', methods=['GET', 'POST'])
def index():

  # The index page has buttons that will send POST requests
  if request.method == "POST":
    print(request.form['waypoint_target'])

    if request.form['waypoint_target'] == 'Waypoint 1':
      hebi_thread.set_waypoint_1()

    if request.form['waypoint_target'] == 'Waypoint 2':
      hebi_thread.set_waypoint_2()

    if request.form['waypoint_target'] == 'Waypoint 3':
      hebi_thread.set_waypoint_3()

    if request.form['waypoint_target'] == 'Waypoint 4':
      hebi_thread.set_waypoint_4()

    if request.form['waypoint_target'] == 'Say Hello!':
      hebi_thread.set_waypoint_say()

  return render_template('index.html')

hebi_thread = HebiThread()
app.run()
