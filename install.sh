# Man wird nach einer Email eingabe gefragt
# Email: trainerai@trainerai.de
# Passwort: esel1212

sudo apt-get update
sudo apt-get install -y postgresql
sudo systemctl is-active postgresql
sudo systemctl is-enabled postgresql
sudo -u postgres psql -c "CREATE USER trainerai WITH PASSWORD 'esel1212';"
sudo -u postgres psql -c "CREATE DATABASE trainerai_db;"
sudo -u postgres psql -c "GRANT ALL PRIVILEGES ON DATABASE trainerai_db to trainerai;"
sudo systemctl restart postgresql
curl https://www.pgadmin.org/static/packages_pgadmin_org.pub | sudo apt-key add
sudo sh -c 'echo "deb https://ftp.postgresql.org/pub/pgadmin/pgadmin4/apt/$(lsb_release -cs) pgadmin4 main" > /etc/apt/sources.list.d/pgadmin4.list && apt update'
sudo apt install pgadmin4
sudo /usr/pgadmin4/bin/setup-web.sh
sudo apt-get -y install nodejs
sudo apt-get -y install npm
npm install sequelize

#Jetzt auf www.http://127.0.0.1/pgadmin4 und einen neuen Server erstellen. Name: trainerai_db, Host = localhost, passwort esel1212
#---- Siehe: Überschrift "Accessing pgAdmin4 Web Interface" auf https://www.tecmint.com/install-postgresql-and-pgadmin-in-ubuntu/
# jetzt "node src/postgres_sequelize/migrate.js" ausführen
