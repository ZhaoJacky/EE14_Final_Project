
// // Generates sine table
// int main(void) {
//     int i; 
//     signed int sine_table[91]; 
//     float sf; 
//     //for 12-bit ADC, [e, 2047(0xFFF)]; 
//     for (i = 0; i <= 90; i++){ 
//         sf = sin(M_PI * i /180); 
//         sine_table[i] = (1 + sf) * 2048; 
//         if(sine_table[i] == 0x1000) 
//             sine_table[i] = 0xFFF; //sin(90) is out of range 
//         } 
//     printf("sine_table"); 
//     for (i = 0; i < 90; i += 5){ 
//         printf("\tDCD\t"); 
//         printf("0x%03x,0x%03x,0x%03x,0x%03x,0x%03x\n", sine_table[i], 
//         sine_table[i+1], sine_table[i+2], sine_table[i+3], sine_table[i+4]); 
//     } 
//     printf("\tDCD\t0x%03x\n", sine_table[90]); 
//     return 0; 
// }
