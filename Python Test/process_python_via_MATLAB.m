% Import Python module
% py.importlib.import_module("C:/Users/haaris/Documents/UC Davis Courses/Spring 2025/EEC 172/Labs/Final Project/Python Test/download_s3_csv.py");

% Define the S3 bucket details
bucket = 'my-lambda-csv-bucket-firemarengo2';
key = 'data.csv';
local_path = 'downloaded_data.csv';

% Call Python function.
% py.download_s3_csv.download_csv(bucket, key, local_path);

% Take downloaded .csv file and read into MATLAB
data = readmatrix(local_path);
disp(data);