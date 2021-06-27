import { Injectable } from '@angular/core';
import { Station } from '../models/station';
import { DataService } from './data.service';

@Injectable({
  providedIn: 'root'
})
export class StationService {

  stations: Station[]


  constructor(private dataService: DataService) {
    let stationBuilded: Station[] | { id: any; name: any; cameraId: number; objDetections: string[]; active: boolean; imageUrl: string; }[] = []
    this.dataService.getStations().subscribe(res => {
      res.stations.forEach((station: any) => {
        stationBuilded.push({
          id: station.id!,
          name: station.name,
          cameraId: 0,
          objDetections: ['towel', 'human', 'dumbbell'],
          active: (Math.random() < 0.5),
          imageUrl: "../../../assets/station-example-1.png"
        });
        console.log(station);
      });
    });
    this.stations = stationBuilded;
  }

  getStations(): Station[] {
    return this.stations;
  }
}
