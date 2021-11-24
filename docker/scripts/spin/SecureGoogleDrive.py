from pydrive.drive import GoogleDrive
from pydrive.auth import GoogleAuth
import pandas as pd


GoogleAuth.DEFAULT_SETTINGS['client_config_file'] = "./docker/scripts/spin/client_secrets.json"
        
gauth = GoogleAuth()
gauth.LocalWebserverAuth()
drive = GoogleDrive(gauth)

# Set the id of the Google Drive folder. You can find it in the URL of the google drive folder.
parent_folder_id = '1blrQs1DdKdHWyDIK_n26rMyuGTergp75'
# Set the parent folder, where you want to store the contents of the google drive folder
parent_folder_dir = './src/AI/spin/extra_data/'

if parent_folder_dir[-1] != '/':
    parent_folder_dir = parent_folder_dir + '/'

# This is the base wget command that we will use. This might change in the future due to changes in Google drive
wget_text = '"wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&amp;confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate \'https://docs.google.com/uc?export=download&amp;id=FILE_ID\' -O- | sed -rn \'s/.*confirm=([0-9A-Za-z_]+).*/\\1\\n/p\')&id=FILE_ID" -O FILE_NAME && rm -rf /tmp/cookies.txt"'.replace('&amp;', '&')


# Get the folder structure
file_dict = dict()
folder_queue = [parent_folder_id]
dir_queue = [parent_folder_dir]
cnt = 0

while len(folder_queue) != 0:
    current_folder_id = folder_queue.pop(0)
    file_list = drive.ListFile({'q': "'{}' in parents and trashed=false".format(current_folder_id)}).GetList()
    
    current_parent = dir_queue.pop(0)
    print(current_parent, current_folder_id)
    for file1 in file_list:
        file_dict[cnt] = dict()
        file_dict[cnt]['id'] = file1['id']
        file_dict[cnt]['title'] = file1['title']
        file_dict[cnt]['dir'] = current_parent + file1['title']

        if file1['mimeType'] == 'application/vnd.google-apps.folder':
            file_dict[cnt]['type'] = 'folder'
            file_dict[cnt]['dir'] += '/'
            folder_queue.append(file1['id'])
            dir_queue.append(file_dict[cnt]['dir'])
        else:
            file_dict[cnt]['type'] = 'file'
            
        cnt += 1
    
pd.DataFrame(file_dict).transpose().head(10)

# Write the bash script
f = open('script.sh', 'w')
file_dict.keys()
f.write('mkdir ' + parent_folder_dir + '\n')
for file_iter in file_dict.keys():
    if file_dict[file_iter]['type'] == 'folder':
        f.write('mkdir ' + file_dict[file_iter]['dir'] + '\n')
    else:
        f.write(wget_text[1:-1].replace('FILE_ID', file_dict[file_iter]['id']).replace('FILE_NAME', file_dict[file_iter]['dir']) + '\n')
f.close()
