import { Injectable } from '@angular/core';
import { Station } from '../models/Station';
import { DataService } from './data.service';

@Injectable({
  providedIn: 'root'
})
export class StationService {

  stations: Station[]


  constructor(private dataService: DataService) {
    let stationBuilded: Station[] = [];
    this.dataService.getStations().subscribe(res => {
      res.forEach((station: any) => {
        stationBuilded.push(station);
        console.log(station);
      });
    });
    this.stations = stationBuilded;
  }

  getStations(): Station[] {
    return this.stations;
  }
}
