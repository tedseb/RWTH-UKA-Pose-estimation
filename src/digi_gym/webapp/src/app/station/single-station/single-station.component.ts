import { Component, Input, OnInit } from '@angular/core';
import { Station } from 'src/app/models/station';

@Component({
  selector: 'app-single-station',
  templateUrl: './single-station.component.html',
  styleUrls: ['./single-station.component.scss']
})
export class SingleStationComponent implements OnInit {

  constructor() {
  }

  ngOnInit(): void {
  }

   @Input()
  station!: Station;
}
