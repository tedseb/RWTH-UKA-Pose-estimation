# Digi-Gym
This repository contains the frontend & backend of the Digi-Gym application for the TrainerAI-Ecosystem. It uses different Node.js modules (server, migrations, database etc.) and relies on a functioning postgres instance, using Angular for the frontend. 

## Postgres SQL

Postgres can be started using following docker command:

```bash
docker run --name digi-gym-db -e POSTGRES_PASSWORD=digi-gym-trainerai-tc -p 5432:5432 -d postgres
```
This starts a container named digi-gym-db running a postgres instance. The password and ports can be changed, but it should be noted that the ports and passwords have to be adjusted in the node.js instances.

## Angular

Angular is a single page application framework. Once the build process is completed, the output folder will contain all static file that are served by the server. The following command has to be run inside the webapp folder:

```bash
ng build --watch --output-path=../dist/
```

The --watch flag sets file watchers for the folder to detect any changes immediately and build again. 


