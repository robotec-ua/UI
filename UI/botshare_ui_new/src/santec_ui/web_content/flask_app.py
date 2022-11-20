#!/usr/bin/env python3
import os
from flask import Flask, render_template, request, redirect, url_for, flash
from werkzeug.utils import secure_filename
from os import walk

app = Flask(__name__)

@app.route('/favicon.ico')
def return_OK():
    return "OK"

@app.route('/')
def index():
    return render_template('index.html')
@app.route('/old')
def old_index():
    return render_template('index_old.html')    



if __name__ == '__main__':
    # app.run(debug=True)
    app.run(host='192.168.0.103', port="5000")
