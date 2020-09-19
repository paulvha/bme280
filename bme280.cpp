/***********************************************************************
 *
 * Extended & Modified for Raspberry Pi August 2018 Paul van Haastrecht
 *
 * Hardware connections: (raspberry B/B+/3/4)
 *
 * components side pin 1 - left
 *
 * I2C connection (as used in this program)
 * VIN       power in : 3V or 5V (pin 1)
 * 3v3       power out (max 100mA) NOT connected
 * GND       GND (pin 6)
 * SCK       I2c SCL (pin 5)        *1
 * sdo       NOT connected
 * sdi       I2C SDA (pin 3)        *1
 * cs        NOT connected
 *
 *      *1 : this is default. In bme280_lib.cpp when using soft_I2C this can
 *           be nearly any GPIO that is wanted
 *
 * By default address is set for 0x77. Add a jumper between SDO and GND
 * to change to 0x76
 *
 * SPI wiring for info only / not supported by this driver!
 * VIN       power in : 3V or 5V
 * 3v3       power out (max 100mA)
 * GND       GND
 * sck       clock
 * SDO       Serial Data out ->  MISO
 * SDI       Serial Data in  ->  MOSI
 * CS        chip select
 *
 * Development environment specifics:
 * Raspberry Pi / linux Jessie release
 *
 * Resources / dependencies:
 * BCM2835 library (http://www.airspayce.com/mikem/bcm2835/)
 * twowire library (https://github.com/paulvha/twowire)
 *
 * *****************************************************************
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **********************************************************************/

#include "BME280.h"
#define  VERSION "September 2020"

#define  BME280_ADDR    0x77    // Default i2C Address

#define  MAXFILE    200
#define  MAXBUF     200
#define  BUFLEN     50
#define  LOOPDELAY  5       // 5 seconds default

typedef struct bmeval
{
    float tempC;        // temperature from the BME280
    float humid;        // humidity from the BME280
    float pressure;     // pressure from the BME280
    float sealevel;     // hold current pressure at sealevel
    float height;       // hold height based on pressure
} bmeval;

typedef struct measure
{
    int       verbose;        // extra information
    uint16_t  loop;           // # of measurement loops
    uint16_t  loop_delay;     // force loop delay (sec)
    char      format[BUFLEN]; // output format
    char      v_save_file[MAXFILE];   // value savefile
    struct bmeval bme;
} measure ;

char progname[20];

/* global constructers */
BME280 MyBme (BME280_ADDR);

/* used as part of p_printf() */
int NoColor=0;

/*********************************************************************
 *  Display in color
 * @param format : Message to display and optional arguments
 *                 same as printf
 * @param level :  1 = RED, 2 = GREEN, 3 = YELLOW 4 = BLUE 5 = WHITE
 *
 *  if NoColor was set, output is always WHITE.
 *********************************************************************/
void p_printf(int level, char *format, ...)
{
    char    *col;
    int     coll=level;
    va_list arg;

    //allocate memory
    col = (char *) malloc(strlen(format) + 20);

    if (NoColor) coll = WHITE;

    switch(coll)
    {
    case RED:
        sprintf(col,REDSTR, format);
        break;
    case GREEN:
        sprintf(col,GRNSTR, format);
        break;
    case YELLOW:
        sprintf(col,YLWSTR, format);
        break;
    case BLUE:
        sprintf(col,BLUSTR, format);
        break;
    default:
        sprintf(col,"%s",format);
    }

    va_start (arg, format);
    vfprintf (stdout, col, arg);
    va_end (arg);

    fflush(stdout);

    // release memory
    free(col);
}

/*********************************************************************
*  generate timestamp
**********************************************************************/
void time_stamp(char *buf)
{
    time_t ltime;
    struct tm *tm ;

    ltime = time(NULL);
    tm = localtime(&ltime);

    static const char wday_name[][4] = {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

    static const char mon_name[][4] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    sprintf(buf, "%.3s %.3s%3d %.2d:%.2d:%.2d %d",
    wday_name[tm->tm_wday],  mon_name[tm->tm_mon],
    tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec,
    1900 + tm->tm_year);
}

/*********************************************************************
*  close hardware and program correctly
**********************************************************************/
void closeout(int val)
{
    /* stop BME280 sensor */
    MyBme.reset();

    /* close I2C channel */
    MyBme.hw_close();

    exit(val);
}

/*********************************************************************
** catch signals to close out correctly
**********************************************************************/
void signal_handler(int sig_num)
{
    switch(sig_num)
    {

        case SIGINT:
        case SIGKILL:
        case SIGABRT:
        case SIGTERM:
        default:
            printf("\nStopping BME280 monitor\n");
            closeout(EXIT_SUCCESS);
            break;
    }
}

/*********************************************************************
** setup signals
**********************************************************************/
void set_signals()
{
    struct sigaction act;

    memset(&act, 0x0,sizeof(act));
    act.sa_handler = &signal_handler;
    sigemptyset(&act.sa_mask);

    sigaction(SIGTERM,&act, NULL);
    sigaction(SIGINT,&act, NULL);
    sigaction(SIGABRT,&act, NULL);
    sigaction(SIGSEGV,&act, NULL);
    sigaction(SIGKILL,&act, NULL);
}



/*********************************************************************
*  usage information
**********************************************************************/

void usage()
{
    p_printf(YELLOW, (char *)
    "%s [options] \n\n"

    "\nBME280: \n\n"
    "-A #       i2C address              (default 0x%02x)\n"
    "-F #       filter coefficient       (default %d)\n"
    "-H #       humidity oversampling    (default %d)\n"
    "-M #       calculate height compared to sealevel pressure\n"
    "-P #       pressure oversampling    (default %d)\n"
    "-R #       run mode                 (default %d)\n"
    "-S #       standby in normal mode   (default %d)\n"
    "-T #       temperature oversampling (default %d)\n"

    "\nprogram: \n\n"
    "-B         no colored output\n"
    "-L #       loop count               (default 0: endless)\n"
    "-D #       delay between loops      (default %d seconds)\n"
    "-O string  output format string\n"
    "-V         verbose & debug messages \n"
    "-W file    save formatted output to file\n"

    "\nI2C settings: \n\n"
    "-i         interface with HARD_I2C  (default software I2C)\n"
    "-I #       I2C speed 1 - 400        (default %d Khz)\n"
    "-s #       SOFT I2C GPIO # for SDA  (default GPIO %d)\n"
    "-d #       SOFT I2C GPIO # for SCL  (default GPIO %d)\n"
    "\n\nVersion %s\n"

    ,progname,BME280_ADDR,
    MyBme.settings.filter, MyBme.settings.humidOverSample,
    MyBme.settings.pressOverSample, MyBme.settings.runMode,
    MyBme.settings.tStandby, MyBme.settings.tempOverSample,
    LOOPDELAY, MyBme.settings.baudrate, DEF_SDA, DEF_SCL, VERSION);
}

/*********************************************************************
 *  perform init (take in account commandline option)
 *  speed of i2C, i2C addresses etc.
 *********************************************************************/
void init_hardware(struct measure *mm)
{
    uint8_t id;

    /* hard_I2C requires  root permission */
    if (MyBme.settings.commInterface == hard_I2C)
    {
        if (geteuid() != 0)
        {
            p_printf(RED,(char *)"You must be super user\n");
            exit(EXIT_FAILURE);
        }
    }

    if(mm->verbose) printf((char *)"initialize BCM2835\n");

    /* set I2C channel */
    if (MyBme.hw_init (MyBme.settings.commInterface ,mm->verbose)) {
        printf("Can't setup i2c pin!\n");
        exit(EXIT_FAILURE);
    }

    if(mm->verbose)
    {
        printf((char *)"set slaveaddres 0x%x\n",MyBme.settings.I2CAddress);
        printf((char *)"set baudrate %dKhz\n",MyBme.settings.baudrate);
        printf((char *)"initialize BME280\n");
    }

    if (MyBme.begin(&id) != SUCCESS)
    {
        p_printf(RED,(char *)"error during starting BME280\n");
        exit(1);
        closeout(EXIT_FAILURE);
    }

    /* BME280 Hardware ID = 0x60 */
    if (id != 0x60)
    {
        p_printf(RED,(char *)"Not a correct Hardware-ID for BME280\n");
        closeout(EXIT_FAILURE);
    }
}

/*********************************************************************
 *  set default variable values
 *********************************************************************/
void init_variables(struct measure *mm)
{
    /* set BME280 */
    MyBme.settings.commInterface = soft_I2C;// I2c communication
    MyBme.settings.I2CAddress = BME280_ADDR;
    MyBme.settings.sda = DEF_SDA;           // default SDA line for soft_I2C
    MyBme.settings.scl = DEF_SCL;           // SCL GPIO for soft_I2C
    MyBme.settings.baudrate = 100;          // set default baudrate
    MyBme.settings.runMode = 3;             // Normal mode = 3, forced = 1 or 2
    MyBme.settings.tStandby = 4;            // 500 ms
    MyBme.settings.filter = 4;              // 16 filter coefficient (highest)
    MyBme.settings.tempOverSample = 5;      // 5 = x 16
    MyBme.settings.pressOverSample = 5;     // 5 = x 16
    MyBme.settings.humidOverSample = 5;     // 5 = x 16

    /* set program instructions */
    mm->verbose = 0;
    mm->loop = 0;
    mm->loop_delay = LOOPDELAY;
    mm->format[0] = 0x0;

    /* reset results */
    mm->bme.sealevel = 0;
    mm->bme.height = 0;
    mm->bme.tempC = 0;
    mm->bme.pressure =0;
    mm->bme.humid=0;
}

/*********************************************************************
 *  Read BME280 for temperature, humidity, pressure and calculate height
 *********************************************************************/
int read_BME280(struct measure *mm)
{
    /* if not in normal running mode.. force it to read one time */
    if (MyBme.settings.runMode != 3)
    {
        if (MyBme.set_run_mode(MyBme.settings.runMode) == ERROR)
        {
            p_printf(RED,(char *)"can not set runmode\n");
            return(ERROR);
        }
    }

    /* Temperature MUST be called first to set t_fine */
    if (MyBme.readTempC(&mm->bme.tempC) == ERROR)
    {
        p_printf(RED,(char *)"can not read Temperature\n");
        return(ERROR);
    }

    if (MyBme.readFloatHumidity(&mm->bme.humid) == ERROR)
    {
        p_printf(RED,(char *)"can not read humidity\n");
        return(ERROR);
    }

    if (MyBme.readFloatPressure(&mm->bme.pressure) == ERROR)
    {
        p_printf(RED,(char *)"can not read pressure\n");
        return(ERROR);
    }

    // Calculate hight in meters
    if (MyBme.readFloatAltitudeMeters(mm->bme.sealevel, &mm->bme.height) == ERROR)
        return(ERROR);

    return(SUCCESS);
}

/*****************************************************************
 * output format can be defined
 *
 *
 * BME results :
 *  T = temperature from BME
 *  H = Humidity from BME
 *  P = Pressure from BME
 *  M = height compared to sealevel
 *
 *
 * Markup:
 *  \l = local time
 *  \t = tab
 *  \s = space
 *  \, = comma
 *  \; = semicolon
 *  \\x = character x is included (x can be any)
 *  \n = new line
 *****************************************************************/
void format_output(struct measure *mm, char *buf)
{
    char    *p,tm[30];

    buf[0] = 0x0;

    /* use default output  if no specific format was requested */
    if (strlen(mm->format) == 0 )
    {
        sprintf(buf, "Temp: %2.2f\tHumidity: %2.2f\n",mm->bme.tempC, mm->bme.humid);
        return;
    }

    p = mm->format;

    while (*p != 0x0 && strlen(buf) < MAXBUF)
    {
        // BME results
        if (*p == 'T') sprintf(tm, " Temp: %2.2f",mm->bme.tempC);
        else if (*p == 'H') sprintf(tm, " Humidity: %2.2f",mm->bme.humid);
        else if (*p == 'P') sprintf(tm, " Pressure: %2.2f",mm->bme.pressure/100);
        else if (*p == 'M') sprintf(tm, " Height: %2.2f",mm->bme.height);

        // markup
        else if (*p == '\\')
        {
            p++;

            if (*p == 't') sprintf(tm, "\t");
            else if (*p == 's') sprintf(tm, " ");
            else if (*p == 'n') sprintf(tm, "\n");
            else if (*p == ',') sprintf(tm, ",");
            else if (*p == ';') sprintf(tm, ";");
            else if (*p == 'l')
            {
                // get timestamp
                time_stamp(tm);
            }
            else if (*p == '\\')
            {
                p++;
                sprintf(tm, "%c",*p);
            }
        }

        // trouble ...
        else
        {
            printf("Illegal character %c in output format string: %s\n", *p, mm->format);

            sprintf(buf, "Temp: %2.2f\tHumidity: %2.2f\n",mm->bme.tempC,mm->bme.humid);

            return;
        }

        // add to buffer
        strcat(buf,tm);

        p++;

    }

    strcat(buf,"\n");
}

/**********************************************************
 *  output values to screen and file (if requested)
 **********************************************************/
int do_output_values(struct measure *mm)
{
    FILE    *fp = NULL ;
    char    buf[MAXBUF];

    // create output string (including Dylos, BME 280 etc. )
    format_output(mm, buf);

    // display output
    p_printf(YELLOW,(char *) "%s",buf);

    // append output to a save_file ?
    if (strlen (mm->v_save_file) > 0)
    {
        if(mm->verbose >1 ) printf("Appending data to file %s\n",mm->v_save_file);

        // save baseline to file
        fp = fopen (mm->v_save_file, "a");

        // Checks if file is open
        if (fp == NULL )
        {
            p_printf(RED,(char *) "Issue with opening output file: %s\n", mm->v_save_file);
            return(ERROR);
        }

        // write ouput
        if (fwrite(buf, sizeof(char), strlen(buf),fp) != strlen(buf))
        {
            p_printf(RED,(char *) "Issue during writing output file: %s\n", mm->v_save_file);
            fclose(fp);
            return(ERROR);
        }

        //close
        fclose(fp);
    }

    return(SUCCESS);
}

/*********************************************************************
 *  main loop to read & display output
 *********************************************************************/

void main_loop(struct measure *mm)
{
    uint16_t lloop;

    /* setup loop count */
    if (mm->loop > 0)   lloop = mm->loop;
    else lloop=1;

    if(mm->verbose) printf((char *)"2 seconds startup time\n");
    sleep(2);

    if(mm->verbose) printf((char *)"starting mainloop\n");

    while (lloop > 0)
    {

        /* read values */
        if (read_BME280(mm) == ERROR) closeout(EXIT_FAILURE);

        /* get output */
        if (do_output_values(mm) == ERROR)  closeout(EXIT_FAILURE);

        /* delay */
        if(mm->verbose) printf((char *)"wait %d seconds\n",mm->loop_delay);
        sleep(mm->loop_delay);

        /* loop count */
        if(mm->loop > 0)    lloop--;
    }
}

/*********************************************************************
 * Parse parameter input (either commandline or file)
 *********************************************************************/

void parse_cmdline(int opt, char *option, struct measure *mm)
{
    switch (opt) {

    case 'A':   // BME280 i2C address
      MyBme.settings.I2CAddress = (int)strtod(option, NULL);

      if (MyBme.settings.I2CAddress != 0x77 && MyBme.settings.I2CAddress != 0x76)
      {
          p_printf(RED,(char *) "incorrect BME280 i2C address %x\n",MyBme.settings.I2CAddress);
          exit(EXIT_FAILURE);
      }
      break;

    case 'B':   // set NO color output
      NoColor = 1;
      break;

    case 'F':   // BME280 filter
      MyBme.settings.filter = (uint8_t)strtod(option, NULL);

      if (MyBme.settings.filter < 0 || MyBme.settings.filter > 4)
      {
          p_printf(RED,(char *) "incorrect BME280 filter  (0 - 5) %d\n",MyBme.settings.filter);
          exit(EXIT_FAILURE);
      }
      break;

    case 'H':   // BME280 Humidity oversampling
      MyBme.settings.humidOverSample = (uint8_t)strtod(option, NULL);

      if (MyBme.settings.humidOverSample < 0 || MyBme.settings.humidOverSample > 5)
      {
          p_printf(RED,(char *) "incorrect BME280 humidity oversampling (0 - 5) %d\n",MyBme.settings.humidOverSample);
          exit(EXIT_FAILURE);
      }
      break;

    case 'I':   // i2C Speed
      MyBme.settings.baudrate = (uint32_t) strtod(option, NULL);

      if (MyBme.settings.baudrate < 1 || MyBme.settings.baudrate > 400)
      {
          p_printf(RED,(char *) "Invalid i2C speed option %d\n",MyBme.settings.baudrate);
          exit(EXIT_FAILURE);
      }
      break;

    case 'L':   // loop count
      mm->loop = (int) strtod(option, NULL);
      break;

    case 'M':   // set pressure sealevel
      if(strlen(option) != 6)
      {
          p_printf(RED,(char *)"invalid pressure must be 6 digits : %s\n",option);
          exit(EXIT_FAILURE);
      }
      mm->bme.sealevel = (float) strtod(option, NULL);
      break;

    case 'O':   // output string
      strncpy(mm->format,option,BUFLEN);
      break;

    case 'P':   // BME280 pressure oversampling
      MyBme.settings.pressOverSample = (uint8_t) strtod(option, NULL);

      if (MyBme.settings.pressOverSample < 0 || MyBme.settings.pressOverSample > 5)
      {
          p_printf(RED,(char *) "incorrect BME280 pressure oversampling (0 - 5) %d\n",MyBme.settings.pressOverSample);
          exit(EXIT_FAILURE);
      }
      break;

    case 'R':   // BME280 run mode
      MyBme.settings.runMode = (uint8_t)strtod(option, NULL);

      if (MyBme.settings.runMode < 1 || MyBme.settings.runMode > 3)
      {
          p_printf(RED,(char *) "incorrect BME280 rumode (1, 2 or 3) %d\n",MyBme.settings.runMode);
          exit(EXIT_FAILURE);
      }
      else if (MyBme.settings.runMode != 3)
            MyBme.settings.filter = 0;  // switch off filter coefficient
      break;

    case 'S':   // BME280 standby in normal mode
      MyBme.settings.tStandby = (uint8_t)strtod(option, NULL);

      if (MyBme.settings.tStandby < 0 || MyBme.settings.tStandby > 7)
      {
          p_printf(RED,(char *) "incorrect BME280 standby in normal(0 - 7) %d\n",MyBme.settings.tStandby);
          exit(EXIT_FAILURE);
      }
      break;

    case 'T':   // BME280 Temperature oversampling
      MyBme.settings.tempOverSample = (uint8_t)strtod(option, NULL);

      if (MyBme.settings.tempOverSample < 0 || MyBme.settings.tempOverSample > 5)
      {
          p_printf(RED,(char *) "incorrect BME280 temperature oversampling (0 - 5) %d\n",MyBme.settings.tempOverSample);
          exit(EXIT_FAILURE);
      }
      break;

    case 'D':   // force loop delay
      mm->loop_delay = (int) strtod(option, NULL);
      break;

    case 'V':   // verbose /debug message
      mm->verbose = 1 ;
      break;

    case 'W':   // save file
      strncpy(mm->v_save_file, option, MAXFILE);
      break;

    case 'i':   // use hardware I2C
      MyBme.settings.commInterface = hard_I2C;
      break;

    case 'd':   // change default SCL line for soft_I2C
      MyBme.settings.scl = (int)strtod(option, NULL);

      if (MyBme.settings.scl < 2 || MyBme.settings.scl == 4 ||
      MyBme.settings.scl > 27 || MyBme.settings.sda == MyBme.settings.scl)
      {
          p_printf(RED,(char *) "invalid GPIO for SCL :  %d\n",MyBme.settings.scl);
          exit(EXIT_FAILURE);
      }
      break;

    case 's':   // change default SDA line for soft_I2C
      MyBme.settings.sda = (int)strtod(option, NULL);

      if (MyBme.settings.sda < 2 || MyBme.settings.sda == 4 ||
      MyBme.settings.sda > 27 || MyBme.settings.sda == MyBme.settings.scl)
      {
          p_printf(RED,(char *) "invalid GPIO for SDA :  %d\n",MyBme.settings.sda);
          exit(EXIT_FAILURE);
      }
      break;
    default:    /* '?' */
        usage();
        exit(EXIT_FAILURE);
    }
}

/*********************************************************************
 *  program starts here
 *********************************************************************/

int main(int argc, char *argv[])
{
    int opt;
    struct measure mm;

    /* initialize default setting */
    init_variables(&mm);

    /* set signals */
    set_signals();

    /* save name for (potential) usage display */
    strncpy(progname,argv[0],20);

    /* parse commandline */
    while ((opt = getopt(argc, argv, "A:F:H:M:P:R:S:T:BC:I:L:D:O:VvW:is:d:")) != -1)
    {
        parse_cmdline(opt, optarg, &mm);
    }

    /* in case of hardware I2C we need to be able to address I2C registers */
    if(MyBme.settings.commInterface == hard_I2C)
    {
        if (geteuid() != 0)
        {
            p_printf(RED,(char *)"You must be super user\n");
            exit(EXIT_FAILURE);
        }
    }

    /* initialize the hardware */
    init_hardware(&mm);

    /* main loop (include command line options)  */
    main_loop(&mm);

    closeout(EXIT_SUCCESS);

    // STOP WALL COMPLAINING
    exit(EXIT_SUCCESS);
}
