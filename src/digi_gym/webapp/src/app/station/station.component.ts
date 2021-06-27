import { Component, OnInit } from '@angular/core';
import { Station } from '../models/station';
import { StationService } from '../services/station.service';

@Component({
  selector: 'app-station',
  templateUrl: './station.component.html',
  styleUrls: ['./station.component.scss']
})
export class StationComponent implements OnInit {


  stations: Station[] = [];
  selectedStation!: Station

  constructor(private stationService: StationService) {
  }

  ngOnInit(): void {
    this.stations = this.stationService.getStations();
    this.selectedStation = this.stations[0];
  }

  selectStation(station: Station) {
    this.selectedStation = station;
  }

}
