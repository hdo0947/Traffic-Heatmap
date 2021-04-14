function generateHeatmap(kitti_id, type, API_KEY)
%% Edit API URL
switch type
    case "hybrid"
        oxts_files = dir(sprintf('../data_set/%s/oxts/data/*.txt',kitti_id));
        

url_ = "https://maps.googleapis.com/maps/api/staticmap?center=49.01668764586999,8.438408125363&zoom=17&size=800x400&markers=color:green%7Clabel:D%7C49.015538872539,8.4363695878495%20&markers=color:red%7Clabel:C%7C49.017836419201,8.4404466628765%20&maptype=hybrid&key=AIzaSyDJSghdJBc-Peznow3d9aQyz4pycisnkjI"; 
    case "satellite"
        pass
    case "roadmap"
        passa
end
%% read image from
url = "https://maps.googleapis.com/maps/api/staticmap?center=49.01668764586999,8.438408125363&zoom=17&size=800x400&markers=color:green%7Clabel:D%7C49.015538872539,8.4363695878495%20&markers=color:red%7Clabel:C%7C49.017836419201,8.4404466628765%20&maptype=hybrid&key=AIzaSyDJSghdJBc-Peznow3d9aQyz4pycisnkjI";
urlwrite(url, 'satellite image.png');
imread(filename)
%%