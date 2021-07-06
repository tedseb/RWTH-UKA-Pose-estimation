import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { any } from 'sequelize/types/lib/operators';
import { Frame } from '../models/Frame';
import { Station } from '../models/Station';
import { FramesService } from '../services/frames.service';
import { StationService } from '../services/station.service';
import { SingleStationComponent } from './single-station/single-station.component';

@Component({
  selector: 'app-station',
  templateUrl: './station.component.html',
  styleUrls: ['./station.component.scss']
})
export class StationComponent implements OnInit {

  stations: Station[] = [];
  selectedStation!: Station
  frameBox!: number[][]
  constructor(private stationService: StationService, private framesService: FramesService) {
  }

  ngOnInit(): void {
    this.stations = this.stationService.getStations();
    this.selectedStation = this.stations[0];
    console.log(this.selectedStation);
  }

  selectStation(station: Station) {
    this.selectedStation = station;
    this.frameBox = [[Math.random() * 100, Math.random()* 100], [Math.random()*1000, Math.random()*1000]]
    let frames = this.framesService.getFrames();
    this.frameBox  =  frames.filter(x => x.camera_station_mapping.station.id == station.id).map(x => x.frame_box)[0];
    console.log(this.frameBox);
  }

}
