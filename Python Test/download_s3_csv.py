import boto3

def download_csv(bucket_name, file_key, local_path):
    s3 = boto3.client('s3')
    s3.download_file(bucket_name, file_key, local_path)
    return f"Downloaded {file_key} into {local_path}"

bucket = 'my-lambda-csv-bucket-firemarengo2'
key = 'data.csv'
local_path = 'downloaded_data.csv'

download_csv(bucket, key, local_path)
