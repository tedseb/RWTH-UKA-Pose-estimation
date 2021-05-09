import { Component, OnInit } from '@angular/core';
import { SpotsService } from '../services/spots.service';
import { Spot } from '../shared/models/spot';



@Component({
  selector: 'app-spots',
  templateUrl: './spots.component.html',
  styleUrls: ['./spots.component.scss']
})
export class SpotsComponent implements OnInit {

  spots: Spot[];

  constructor(private spotsService: SpotsService) {
    this.spots = [];
  }

  ngOnInit(): void {
    this.spotsService.spots.subscribe(spots => {this.spots = spots});
  }

}
