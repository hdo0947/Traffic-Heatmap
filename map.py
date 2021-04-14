import urllib

url = "https://maps.googleapis.com/maps/api/staticmap?center=49.01668764586999,8.438408125363&zoom=17&size=800x400&markers=color:green%7Clabel:D%7C49.015538872539,8.4363695878495%20&markers=color:red%7Clabel:C%7C49.017836419201,8.4404466628765%20&maptype=hybrid&key=AIzaSyDJSghdJBc-Peznow3d9aQyz4pycisnkjI"
with urllib.request.urlopen(url) as req:
    rr = req.read()
    with open("hello.png", 'wb') as hf:
        hf.write(rr)
