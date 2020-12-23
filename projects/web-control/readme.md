# Arm controlled through publicly accessible web server

## Description
This demo will setup a 6-Dof Arm to move to pre-determined waypoints at the click of a button on a website. A simple webpage is designed, which has text at the top, a youtube livestream embedded video on the left, and a series of buttons on the right which send the arm to different waypoints. The arm is controlled in a separate thread, which is started at the same time as the flask server. The server will handle POST requests from the webpage, and translate them to function calls for the arm.

## Included Files
* templates/index.html
* static/css/style.css
* web-control-server.py
* HebiThread.py
* readme.md (this file)

## Dependencies
* Python 3 Flask
* CloudFlared, Free Version - [link](https://developers.cloudflare.com/argo-tunnel/trycloudflare)

## Running the Demo
**Launch the local server**
> `python3 web-control-server.py`

See the pages locally in your browser at http://127.0.0.1:5000.

**Take the server online/public**
> `cloudflared tunnel --url http://localhost:5000`

#The Youtube video livestream
I used OBS to livestream directly onto the HEBI youtube channel, where the livestream was unlisted. **You will need to update `index.html` with the appropriate new URL for the youtube livestream before you can take the page live**. When you have the youtube video live, just click share -> embed -> copy the url from the embed code there and replace the URL in `index.html`.

## General Steps / Notes
* Design the webpage in index.html. Set buttons to send the same POST request with different values
* Setup arm behavior API in HebiThread.py
* In web-control-server.py, check for different POST values, and match them with the appropriate functions in the HEBI Thread.
* Run the Flask server locally to test things out in your browser.
* When ready to go public, its super easy. Just use the command described above and it directly gives a link to send out to others.
* Take note, that URL will be very weird. Consider the paid service from cloudflared, or a different service, to avoid the weird URLs.
