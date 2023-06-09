from pydrive.drive import GoogleDrive
from pydrive.auth import GoogleAuth
import pandas as pd


GoogleAuth.DEFAULT_SETTINGS['client_config_file'] = "./docker/scripts/largeFiles/client_secrets.json"
        
gauth = GoogleAuth()
gauth.LocalWebserverAuth()
drive = GoogleDrive(gauth)


# folder structure = <google_drive_folder_id>: <local_path>
folder_structure = {
    "1prCzNSakKbOuIzNJwSEUSGDdeIWrdIRl": "./src/AI/metrabs/models/",
}


# This is the base wget command that we will use. This might change in the future due to changes in Google drive
wget_text = '"wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&amp;confirm=$(wget --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate \'https://docs.google.com/uc?export=download&amp;id=FILE_ID\' -O- | sed -rn \'s/.*confirm=([0-9A-Za-z_]+).*/\\1\\n/p\')&id=FILE_ID" -O FILE_NAME && rm -rf /tmp/cookies.txt"'.replace('&amp;', '&')


# Get the folder structure
file_dict = dict()
folder_queue = list(folder_structure.keys())
dir_queue = list(folder_structure.values())
dir_queue_copy = dir_queue.copy()
cnt = 0

for i, folder in enumerate(dir_queue):
    if folder[-1] != '/':
        folder_queue[i] = folder + '/'

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
print(dir_queue_copy)
for folder in dir_queue_copy:
    f.write('mkdir ' + folder + '\n')

for file_iter in file_dict.keys():
    if file_dict[file_iter]['type'] == 'folder':
        f.write('mkdir ' + file_dict[file_iter]['dir'] + '\n')
    else:
        f.write(wget_text[1:-1].replace('FILE_ID', file_dict[file_iter]['id']).replace('FILE_NAME', file_dict[file_iter]['dir']) + '\n')
f.close()
