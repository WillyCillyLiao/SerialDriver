/* 
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the 
 * United States National  Science Foundation and the Department of Energy.  
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national 
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xtack.sandia.gov/hobbes
 *
 * Copyright (c) 2015, Kyle C. Hale <kh@u.northwestern.edu>
 * Copyright (c) 2015, The V3VEE Project  <http://www.v3vee.org> 
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Author: Kyle C. Hale <kh@u.northwestern.edu>
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 *
 * Parts of this file were taken from the GeekOS teaching OS 
 */
#include <nautilus/nautilus.h>
#include <nautilus/fmtout.h>
#include <nautilus/idt.h>
#include <nautilus/irq.h>
#include <nautilus/cpu.h>
#include <nautilus/shutdown.h>
#include <dev/serial.h>
#include <nautilus/vc.h>
#include <dev/vga.h>


extern int vprintk(const char * fmt, va_list args);

static spinlock_t serial_lock; /* for SMP */
static uint8_t serial_device_ready = 0;
uint16_t serial_io_addr = 0;
uint_t serial_print_level;
static uint8_t com_irq;

static int 
serial_irq_handler (excp_entry_t * excp,
                    excp_vec_t vec)
{
  char rcv_byte;
  char irq_id;
  char dataReady;
  char error;
  //printk("IRQ");
  vga_print("IRQ\n");
  irq_id = inb(serial_io_addr + 2);// read register 2, interrupt identification
  irq_id = irq_id & 0x0f;
  
  //nk_vc_printf("IRQ_ID = %d\n", irq_id);
  
  //nk_vc_print(&irq_id);
  //serial_putchar(irq_id);
  if (irq_id==6) {
    //errors occur
    vga_print("error\n");
    error = inb(serial_io_addr + 5);
    if(error == 0x1)   {
      //serial_print("Overrun Error(OE)\n");
    }  else if(error == 0x2) {
      //serial_print("Parity Error(PE)\n");
    } else if(error == 0x3) 	{
      //serial_print("Framing Error(FE)\n");
    }  else if(error == 0x4) 	{
      //serial_print("Break Interrupt(BI)\n");
    } else if(error == 0x7)     {
      //serial_print("Error in RCVR FIFO\n");
    }
    else {
    }
  } else if (irq_id == 4) { 
    // irq_id!=0, then no interrupt pending
    // COM1_3_IRQ = 4; COM2_4_IRQ = 3
    //nk_vc_printf("IRQ_ID = %d\n", irq_id);
    vga_print("RECEIVE\n");
    dataReady = inb(serial_io_addr+5);
    if (dataReady & 0x01) {
    recv:
      rcv_byte = inb(serial_io_addr + 0); // read register 0, receiver buffer register
      
      switch (rcv_byte)  {
      case 'k' :
	{
	  //serial_print("Rebooting Machine\n");
	  reboot();
	  break;
	}
	
      case 'p' :
	{
	  //serial_print("Manually invoking panic\n");
	  panic();
	  break;
	}
      case 's' :
	acpi_shutdown();
	break;
      default:// the most case
	//serial_putchar(rcv_byte);
	//get it displayed on nautilus
	//rcv_byte=serial_getchar();
	//serial_puts(&rcv_byte);
	//serial_print("%s",rcv_byte );
	nk_vc_handle_input_serial(rcv_byte);
	break;
      }
    }
  } else if(irq_id==12) {
    //timeout
    vga_print("timeout\n");
    goto recv;
    //outb(0x07, serial_io_addr+2);// FIFO control reg enable 1 byte FIFO and reset rcv and transmit FIFO
  } else if (irq_id==2) {
    //transmitter holding register empty
    vga_print("TRANSMIT\n");
    uchar_t c;
    c = nk_vc_handle_output_serial();// get character from SerialOut in vc.c
    
    if (c == NO_CHAR ){
      return 0;
    }
    //print for debug
    nk_vc_printf("Dequeue %c from Serial Out successfully !\n", c);
    
    outb(c,serial_io_addr); // put the dequeued character into the transmitter holding reg
    
    //print for debug
    nk_vc_printf("Put %c into transmitter FIFO\n", c);
    
    //nk_vc_printf("char:%c.", c);
  } else {
      vga_print("else\n");
  }
  IRQ_HANDLER_END();
  return 0;
}


void 
serial_init_addr (uint16_t io_addr) 
{
  serial_io_addr = io_addr;


  //  io_adr = 0x3F8;	/* 3F8=COM1, 2F8=COM2, 3E8=COM3, 2E8=COM4 */
  outb(0x80, io_addr + 3);// write to register 3, DLAB=1

  // 115200 /* 115200 / 12 = 9600 baud */
  outb(1, io_addr + 0);
  outb(0, io_addr + 1);//  wrtite divisor latch = 0x0001, that is, baud rate = 9600

  /* 8N1 */
  outb(0x03, io_addr + 3);// write register 3, 
  //						set word length select= 0011, that is, 8 bit character
  //						set DLAB=0

  /* interrupts enabled or disenabled */
  //  outb(0, io_addr + 1);
  //outb(0x01, io_addr + 1);// write reg1(interrupt enable reg),only enable receive data interrupt
  //outb(0x07, io_addr+1);//enable receive data interrupt, transmit holding empty interrupt and receiver line status interrupt
  outb(0x03, io_addr + 1);// enable receive data interrupt, transimit holding empty interrupt
  //outb(0x02, io_addr + 1);// enable transmit holding empty interrupt

  /* turn on FIFO, if any */
  //outb(0, io_addr + 2);// write reg2, FIFO control reg disable
  //outb(0x81, io_addr+2); // FIFO control reg enable 8 byte FIFO
  //outb(0x01, io_addr+2); // FIFO control reg enable 1 byte FIFO
  outb(0x07, io_addr+2); // FIFO control reg enable 1 byte FIFO and reset rcv and transmit FIFO

  /* loopback off, interrupts (Out2) off, Out1/RTS/DTR off */
  //  outb(0, io_addr + 4);
  
  // enable interrupts (bit 3)
  outb(0x08, io_addr + 4);// write reg4,(MODEM control reg) output 2???????



}

void 
serial_putchar (uchar_t c)
{
    //  static unsigned short io_adr;
    if (serial_io_addr==0) { // has not init
        return;
    }

    if (!serial_device_ready) {
        return;
    }

    int flags = spin_lock_irq_save(&serial_lock);

    if (c == '\n') { 

        /* wait for transmitter ready */
        while( (inb(serial_io_addr + 5) & 0x40) == 0);
        //read reg5(line status reg),if transmitter empty(transmitter holding reg & transimitter shift reg)

        /* send char */
        outb('\r', serial_io_addr + 0);// write to transmitter holding register

        /* wait for transmitter ready */
    } 

    while( (inb(serial_io_addr + 5) & 0x40) == 0);

    /* send char */
    //write: write data to device
    //receive: receive data from device
    outb(c, serial_io_addr + 0);// transmit from nautilus to device

    spin_unlock_irq_restore(&serial_lock, flags);
}


void 
serial_putlnn (const char * line, int len) 
{
  while ((*line) && len--) {
      serial_putchar(*line);
  }

  serial_putchar('\n');
}


void 
serial_write (const char *buf) 
{
  while (*buf) {
      serial_putchar(*buf);
      ++buf;
  }
}

void 
serial_puts( const char *buf)// serial_put = serial_putslnn (why need two function?) 
{
  serial_write(buf);
  serial_putchar('\n');
}


void 
serial_print_hex (uchar_t x)
{
  uchar_t z;
  
  z = (x >> 4) & 0xf;// get the highest 4 bits
  serial_print("%x", z);

  z = x & 0xf;// get lowest 4 bits
  serial_print("%x", z);
}


void 
serial_mem_dump (uint8_t * start, int n)
{
    int i, j;

    for (i = 0; i < n; i += 16) { // one loop one line

        serial_print("%8x", (ulong_t)(start + i));

        for (j = i; j < i + 16 && j < n; j += 2) { 

            serial_print(" ");
            serial_print_hex(*((uchar_t *)(start + j)));
            if ((j + 1) < n) { 
                serial_print_hex(*((uchar_t *)(start + j + 1)));
            }

        }

        serial_print(" ");

        for (j=i; j<i+16 && j<n;j++) {
            serial_print("%c", ((start[j] >= 32) && (start[j] <= 126)) ? start[j] : '.');
            // if it can be found on keyboard, print it, or print".".
        }
        serial_print("\n");
        // 2 characters a group, 8 groups a line, one line has one head(1st character) and one summary(all)

    }

}


static struct Output_Sink serial_output_sink;

static void 
Serial_Emit (struct Output_Sink * o, int ch) 
{ 
  serial_putchar((uchar_t)ch); 
}

static void 
Serial_Finish (struct Output_Sink * o) { return; }


void 
__serial_print (const char * format, va_list ap) 
{
    uint8_t flags;

    if (serial_device_ready) {
        flags = spin_lock_irq_save(&serial_lock);
        Format_Output(&serial_output_sink, format, ap);
        spin_unlock_irq_restore(&serial_lock, flags);
    } else {
        vprintk(format, ap);
    }
}


void 
serial_print (const char * format, ...)
{
  va_list args;
  uint8_t iflag = irq_disable_save();

  va_start(args, format);
  __serial_print(format, args);
  va_end(args);

  irq_enable_restore(iflag);
}


void 
serial_print_list (const char * format, va_list ap) 
{
  uint8_t iflag = irq_disable_save();
  __serial_print(format, ap);
  irq_enable_restore(iflag);
}


void 
serial_printlevel (int level, const char * format, ...) 
{
  if (level > serial_print_level) {
    va_list args;
    uint8_t iflag = irq_disable_save();
    
    va_start(args, format);
    __serial_print(format, args);
    va_end(args);
    
    irq_enable_restore(iflag);   
  }
}


uint8_t 
serial_get_irq (void)
{
    return com_irq;
}

void 
serial_init (void) 
{
  serial_print_level = SERIAL_PRINT_DEBUG_LEVEL;

  spinlock_init(&serial_lock);

  serial_output_sink.Emit = &Serial_Emit;
  serial_output_sink.Finish = &Serial_Finish;

#if NAUT_CONFIG_SERIAL_PORT == 1 
  serial_init_addr(COM1_ADDR);
  register_irq_handler(COM1_3_IRQ, serial_irq_handler, NULL);
  com_irq = COM1_3_IRQ;
#elif NAUT_CONFIG_SERIAL_PORT == 2 
  serial_init_addr(COM2_ADDR);
  register_irq_handler(COM2_4_IRQ, serial_irq_handler, NULL);
  com_irq = COM2_4_IRQ;
#elif NAUT_CONFIG_SERIAL_PORT == 3 
  serial_init_addr(COM3_ADDR);
  register_irq_handler(COM1_3_IRQ, serial_irq_handler, NULL);
  com_irq = COM1_3_IRQ;
#elif NAUT_CONFIG_SERIAL_PORT == 4
  serial_init_addr(COM4_ADDR);
  register_irq_handler(COM2_4_IRQ, serial_irq_handler, NULL);
  com_irq = COM2_4_IRQ;
#else
#error Invalid serial port
#endif

  serial_device_ready = 1;
}

/*uchar_t 
serial_getchar (void)
{
	uchar_t c;
    //  static unsigned short io_adr;
    if (serial_io_addr==0) { // has not init
        c=0;
        return c;
    }

    if (!serial_device_ready) {
        c=0;
        return c;
    }

    int flags = spin_lock_irq_save(&serial_lock);

    while( (inb(serial_io_addr + 5) & 0x01) == 0);

    
    //write: write data to device
    //receive: receive data from device
  
    c = inb(serial_io_addr + 0);// receive from device to nautilus


    if (c == '\n') { 

        
        while( (inb(serial_io_addr + 5) & 0x01) == 0);
        //read reg5(line status reg),if data is ready

        
        outb('\r', serial_io_addr + 0);// write to receiver buffer register

       
    } 

    
    spin_unlock_irq_restore(&serial_lock, flags);
    return c;
}
*/

