#include "UI.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "LCD.h"
#include "colors.h"
#include "ST7789.h"
#include "T6963.h"
#include "font.h"
#include "control.h"
#include "FX.h"
#include "timers.h"

#include <stdlib.h>

extern int g_set_current_array[480]; 
extern int g_measure_current_array[480];

// Green fields = read/write
// Orange fields = read-only
UI_FIELD_T Fields[] = {
//	{"50 ", "", "", NULL, NULL, {0,1}, 
//	&green, &black, 0, 0, 1, 1,NULL},
//	{"40 ", "", "", NULL, NULL, {0,2}, 
//	&green, &black, 0, 0, 1, 1,NULL},
//	{"30 ", "", "", NULL, NULL, {0,3}, 
//	&green, &black, 0, 0, 1, 1,NULL},
//	{"20 ", "", "", NULL, NULL, {0,4}, 
//	&green, &black, 0, 0, 1, 1,NULL},
//	{"10 ", "", "", NULL, NULL, {0,5}, 
//	&green, &black, 0, 0, 1, 1,NULL},
//	{"-10", "", "", NULL, NULL, {0,6}, 
//	&green, &black, 0, 0, 1, 1,NULL},
	{"Duty Cycle  ", "ct", "", (volatile int *)&g_duty_cycle, NULL, {0,7}, 
	&green, &black, 1, 0, 0, 0,Control_DutyCycle_Handler},
	{"Enable Ctlr ", "", "", (volatile int *)&g_enable_control, NULL, {0,8}, 
	&green, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"Enable Flash", "", "", (volatile int *)&g_enable_flash, NULL, {0,9}, 
	&green, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"T_flash_prd ", "ms", "", (volatile int *)&g_flash_period, NULL, {0,10}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},		
	{"T_flash_on  ", "ms", "", (volatile int *)&g_flash_duration, NULL, {0,11}, 
	&green, &black,  1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_set       ", "mA", "", (volatile int *)&g_set_current, NULL, {0,12}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_set_peak  ", "mA", "", (volatile int *)&g_peak_set_current, NULL, {0,13}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_measured  ", "mA", "", (volatile int *)&g_measured_current, NULL, {0,14}, 
	&orange, &black, 1, 0, 1, 1, NULL},
};

UI_SLIDER_T Slider = {
	0, {0,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {UI_SLIDER_WIDTH-1,LCD_HEIGHT-1}, 
	{119,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {119,LCD_HEIGHT-1}, &white, &dark_gray, &light_gray
};

int UI_sel_field = -1;

void UI_Update_Field_Values (UI_FIELD_T * f, int num) {
	int i;
	for (i=0; i < num; i++) {
		snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%4d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
		f[i].Updated = 1;
	}	
}

void UI_Update_Volatile_Field_Values(UI_FIELD_T * f) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		if (f[i].Volatile) {
			snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%4d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
			f[i].Updated = 1;
		}
	}
}

void UI_Draw_Fields(UI_FIELD_T * f, int num){
	int i;
	COLOR_T * bg_color;
	for (i=0; i < num; i++) {
		if ((f[i].Updated) || (f[i].Volatile)) { // redraw updated or volatile fields
			f[i].Updated = 0;
			if ((f[i].Selected) && (!f[i].ReadOnly)) {
				bg_color = &dark_red;
			} else {
				bg_color = f[i].ColorBG;
			}
			LCD_Text_Set_Colors(f[i].ColorFG, bg_color);
			LCD_Text_PrintStr_RC(f[i].RC.Y, f[i].RC.X, f[i].Buffer);
		}
	}
}

void UI_Draw_Slider(UI_SLIDER_T * s) {
	static int initialized=0;
	
	if (!initialized) {
		LCD_Fill_Rectangle(&s->UL, &s->LR, s->ColorBG);
		initialized = 1;
	}
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorBG); // Erase old bar
	
	s->BarUL.Y = s->UL.Y;
	s->BarLR.Y = s->LR.Y;
	s->BarUL.X = (s->LR.X - s->UL.X)/2 + s->Val;
	s->BarLR.X = s->BarUL.X + UI_SLIDER_BAR_WIDTH/2;
	s->BarUL.X -= UI_SLIDER_BAR_WIDTH/2;
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorFG); // Draw new bar
}

int UI_Identify_Field(PT_T * p) {
	int i, t, b, l, r;

	if ((p->X >= LCD_WIDTH) || (p->Y >= LCD_HEIGHT)) {
		return -1;
	}

	if ((p->X >= Slider.UL.X) && (p->X <= Slider.LR.X) 
		&& (p->Y >= Slider.UL.Y) && (p->Y <= Slider.LR.Y)) {
		return UI_SLIDER;
	}
  for (i=0; i<UI_NUM_FIELDS; i++) {
		l = COL_TO_X(Fields[i].RC.X);
		r = l + strlen(Fields[i].Buffer)*CHAR_WIDTH;
		t = ROW_TO_Y(Fields[i].RC.Y);
		b = t + CHAR_HEIGHT-1;
		
		if ((p->X >= l) && (p->X <= r) 
			&& (p->Y >= t) && (p->Y <= b) ) {
			return i;
		}
	}
	return -1;
}

void UI_Update_Field_Selects(int sel) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		Fields[i].Selected = (i == sel)? 1 : 0;
	}
}

void UI_Process_Touch(PT_T * p) {  // Called by Thread_Read_TS
	int i;
	
	i = UI_Identify_Field(p);
	if (i == UI_SLIDER) {
		Slider.Val = p->X - (Slider.LR.X - Slider.UL.X)/2; // Determine slider position (value)
		if (UI_sel_field >= 0) {  // If a field is selected...
			if (Fields[UI_sel_field].Val != NULL) {
				if (Fields[UI_sel_field].Handler != NULL) {
					(*Fields[UI_sel_field].Handler)(&Fields[UI_sel_field], Slider.Val); // Have the field handle the new slider value
				}
				UI_Update_Field_Values(&Fields[UI_sel_field], 1);
			}
		}
	} else if (i>=0) {
		if (!Fields[i].ReadOnly) { // Can't select (and modify) a ReadOnly field
			UI_sel_field = i;
			UI_Update_Field_Selects(UI_sel_field);
			UI_Update_Field_Values(Fields, UI_NUM_FIELDS);
			Slider.Val = 0; // return to slider to zero if a different field is selected
		}
	} 
}
void UI_Draw_Background(void){
	int x, y, r;
	PT_T p1, p2;
	
	// vertical lines
	p1.Y = 5;
	p2.Y = 125;
	for (x=5; x<=LCD_WIDTH; x += 40) {
		p1.X = p2.X = x;
		LCD_Draw_Line(&p1, &p2, &light_gray);
	}
	
	// horizontal line
	p1.X = 5;
	p2.X = 240;
	for (y=15; y<=125; y += 20) {
		p1.Y = p2.Y = y;
		LCD_Draw_Line(&p1, &p2, &light_gray);
	}
}

void UI_Draw_set_current(int set_current[480], int measure_current[480]){
	static int x_pos = 0;
	int y_pos;
	static PT_T p1,p2;
	PT_T p3,p4;
	COLOR_T c;
	
	int y_measure_pos;
	static PT_T p_measure1,p_measure12;	
	
	c.R = 0;
	c.G = 0;
	c.B = 0;
	
	for(int i=0;i<480;i=i+2){
		if(x_pos < 240){
			y_pos = set_current[i];
			for(int k=1;k<4;k++){
				if(y_pos < set_current[i+k])
					y_pos = set_current[i+k];
			}	
			if((80>= y_pos) && (y_pos >= -20)){
				p1.Y = 15 + (80-y_pos);	
			}
			else if((y_pos >80)&& (y_pos < 95)){
				p1.Y = abs(15 - (y_pos-80));
			}
			else if(y_pos > 95){
				p1.Y = 0;
			}	
			else
				return;
			
			y_measure_pos = measure_current[i];
			for(int k=1;k<4;k++){
				if(y_measure_pos < measure_current[i+k])
					y_measure_pos = measure_current[i+k];
			}	 
			if((80>= y_measure_pos) && (y_measure_pos >= -20)){
				p_measure1.Y = 15 + (80-y_measure_pos);	
			}
			else if((y_measure_pos >80) && (y_measure_pos <= 95)){
				p_measure1.Y = abs(15 - (y_measure_pos-80));
			}	
			else if(y_measure_pos > 95){
				p_measure1.Y = 0;
			}
			else
				return;
			p_measure1.X = x_pos;		
			p1.X = x_pos;	
			if(x_pos==0)
				p2=p1;
			LCD_Draw_Line(&p2,&p1,&blue);	
			LCD_Plot_Pixel(&p_measure1, &yellow);	
			x_pos++;	
			p2=p1;
		}
		else{
			p3.X=0;
			p3.Y=0;
			p4.X=240;
			p4.Y=125;
			
			LCD_Fill_Rectangle(&p3,&p4,&c);
			UI_Draw_Background();
			x_pos=0;
						y_pos = set_current[i];
			for(int k=1;k<2;k++){
				if(y_pos < set_current[i+k])
					y_pos = set_current[i+k];
			}	
			if((80>= y_pos) && (y_pos >= -20)){
				p1.Y = 15 + (80-y_pos);	
			}
			else if((y_pos >80)&& (y_pos < 95)){
				p1.Y = abs(15 - (y_pos-80));
			}
			else if(y_pos > 95){
				p1.Y = 0;
			}	
			else
				return;
			
			y_measure_pos = measure_current[i];
			for(int k=1;k<2;k++){
				if(y_measure_pos < measure_current[i+k])
					y_measure_pos = measure_current[i+k];
			}	 
			if((80>= y_measure_pos) && (y_measure_pos >= -20)){
				p_measure1.Y = 15 + (80-y_measure_pos);	
			}
			else if((y_measure_pos >80) && (y_measure_pos <= 95)){
				p_measure1.Y = abs(15 - (y_measure_pos-80));
			}	
			else if(y_measure_pos > 95){
				p_measure1.Y = 0;
			}
			else
				return;
			p_measure1.X = x_pos;		
			p1.X = x_pos;	
			if(x_pos==0)
				p2=p1;
			LCD_Draw_Line(&p2,&p1,&blue);	
			LCD_Plot_Pixel(&p_measure1, &yellow);	
			x_pos++;	
			p2=p1;
		}
	}
}


void UI_Draw_Screen(int first_time) { // Called by Thread_Update_Screen
	static uint32_t counter=0;
	char buffer[32];
	
	if (first_time) {
		UI_Update_Field_Values(Fields, UI_NUM_FIELDS);
	}
	UI_Update_Volatile_Field_Values(Fields);
	UI_Draw_Fields(Fields, UI_NUM_FIELDS);
	UI_Draw_Slider(&Slider);	
	LCD_Text_Set_Colors(&white, &black);
	if (first_time) {
//			LCD_Text_PrintStr_RC(0, 6, "screen redraws");
		UI_Draw_Background();		
	}
//	sprintf(buffer, "%5d", counter++);
	LCD_Text_PrintStr_RC(0, 0, buffer);
	UI_Draw_set_current(g_set_current_array,g_measure_current_array);
//	UI_Draw_measure_current(g_measure_current_array);
	
}
