import { Injectable } from '@angular/core';
import { Station } from '../models/station';

@Injectable({
  providedIn: 'root'
})
export class StationService {

  stations: Station[]


  constructor() {
    this.stations = [
      {stationId: 1, stationDescription: "Power Rack", cameraId: 1, objDetections: ['towel', 'plant', 'human'], active: true, imageUrl: "../../../assets/station-example-1.png"},
      {stationId: 2, stationDescription: "Test-Station", cameraId: 4, objDetections: ['dumbbell', 'weight', 'human'], active: false, imageUrl: "../../../assets/station-example-2.png"}
    ]
  }

  getStations(): Station[] {
    return this.stations;
  }
}
