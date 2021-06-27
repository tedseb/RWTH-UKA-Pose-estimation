import { Component, OnInit } from '@angular/core';
import { multi } from './data';

@Component({
  selector: 'app-statistics',
  templateUrl: './statistics.component.html',
  styleUrls: ['./statistics.component.scss']
})
export class StatisticsComponent implements OnInit {

  multi: any[] = [];
  view: [number, number] = [700, 400];

  // options
  showXAxis: boolean = true;
  showYAxis: boolean = true;
  gradient: boolean = true;
  showLegend: boolean = true;
  showXAxisLabel: boolean = true;
  xAxisLabel: string = 'Usage';
  showYAxisLabel: boolean = true;
  yAxisLabel: string = 'Station';
  legendTitle: string = 'Days';

  colorScheme = {
    domain:  ["#7400b8","#4ea8de","#80ffdb"]
  };


  constructor() {
    Object.assign(this, { multi });
  }

  ngOnInit(): void {
  }



 onSelect(data: any): void {
    console.log('Item clicked', JSON.parse(JSON.stringify(data)));
  }

  onActivate(data: any): void {
    console.log('Activate', JSON.parse(JSON.stringify(data)));
  }

  onDeactivate(data: any): void {
    console.log('Deactivate', JSON.parse(JSON.stringify(data)));
  }
}
