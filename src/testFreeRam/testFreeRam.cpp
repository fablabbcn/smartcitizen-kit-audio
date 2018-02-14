// This sketch is meant for reading audio over 

#include <Arduino.h>

// ///// OUTPUT
int timer = 0;
int *testBuffer4, *testBuffer5 = 0;
int A = 0;

class TestClassDynamic
{
    public:

        TestClassDynamic(int bufferSize)
        {
            _testBuffer = NULL;
            _bufferSize = bufferSize;
        }

        ~TestClassDynamic(){
            free(_testBuffer);
        }

        void init(){
            _testBuffer = calloc(_bufferSize, sizeof(int));
        }

        int fill(){
            int* testB = (int*) _testBuffer;
            for (int i=0;i<_bufferSize;i++){
                *testB = i;
                testB++;
            }
            return testB[_bufferSize/2];
        }

        void end(){
            free (_testBuffer);
        }

    private:
        void* _testBuffer;
        int _bufferSize;

    private:
        uint32_t FreeRamMem() {
                uint32_t stackTop;
                uint32_t heapTop;

                // Current position of the stack
                stackTop = (uint32_t) &stackTop;

                // Current position of heap
                void* hTop = malloc(1);
                heapTop = (uint32_t) hTop;
                free(hTop);

                // The difference is the free, available ram
                return stackTop - heapTop;
        }
};

class TestClassStatic
{
    public:

        TestClassStatic(int bufferSize)
        {
            _testBuffer [bufferSize];
            _bufferSize = bufferSize;
        }

        ~TestClassStatic(){
        }

        void init(){
        }

        int fill(){
            for (int i=0;i<_bufferSize;i++){
                _testBuffer[i] = i;
            }
            return _testBuffer[_bufferSize/2];
        }

        void end(){
            free (_testBuffer);
        }

    private:
        int _bufferSize;
        int _testBuffer[];
    private:
        uint32_t FreeRamMem() {
                uint32_t stackTop;
                uint32_t heapTop;

                // Current position of the stack
                stackTop = (uint32_t) &stackTop;

                // Current position of heap
                void* hTop = malloc(1);
                heapTop = (uint32_t) hTop;
                free(hTop);

                // The difference is the free, available ram
                return stackTop - heapTop;
        }
};

class NoiseClass{
    public:

        NoiseClass(int bufferSize)
        {
            _bufferSize = bufferSize;
        }

        ~NoiseClass(){
            free(_testNoiseDyn);
        }

        void init(){
            _testNoiseDyn = new TestClassDynamic(_bufferSize);
            _testNoiseDyn->init();
            _testNoiseDyn->fill();
            _testNoiseDyn->end();
            delete _testNoiseDyn;        
        }
        void end(){
            // Rien de rien
        }

        int getDyn(){
            SerialUSB.println("Get Dynamic");
            SerialUSB.println(FreeRamMem());
            _testNoiseDyn = new TestClassDynamic(_bufferSize);
            SerialUSB.println(FreeRamMem());
            _testNoiseDyn->init();
            SerialUSB.println(FreeRamMem());
            int _testValue = _testNoiseDyn->fill();
            SerialUSB.println(FreeRamMem());
            _testNoiseDyn->end();
            SerialUSB.println(FreeRamMem());
            return _testValue;
        }

        int getSt(){
            SerialUSB.println("Get Static");
            SerialUSB.println(FreeRamMem());
            _testNoiseSt = new TestClassStatic(_bufferSize);
            SerialUSB.println(FreeRamMem());
            _testNoiseSt->init();
            SerialUSB.println(FreeRamMem());
            int _testValue = _testNoiseSt->fill();
            SerialUSB.println(FreeRamMem());
            _testNoiseSt->end();
            SerialUSB.println(FreeRamMem());
            return _testValue;
        }



    private:
        int _bufferSize;
        TestClassDynamic* _testNoiseDyn;
        TestClassStatic* _testNoiseSt;

        uint32_t FreeRamMem() {
            uint32_t stackTop;
            uint32_t heapTop;

            // Current position of the stack
            stackTop = (uint32_t) &stackTop;

            // Current position of heap
            void* hTop = malloc(1);
            heapTop = (uint32_t) hTop;
            free(hTop);

            // The difference is the free, available ram
            return stackTop - heapTop;
        }
};

uint32_t FreeRamMem() {
    uint32_t stackTop;
    uint32_t heapTop;

    // Current position of the stack
    stackTop = (uint32_t) &stackTop;

    // Current position of heap
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram
    return stackTop - heapTop;
}

void allocateBuffer(int bufferSize) {
    int* testBuffer3 = (int*) calloc(bufferSize, sizeof(int));  
    for (int i=0;i<10;i++){
        testBuffer3 [i]=i*2;
    }

    SerialUSB.println(FreeRamMem());


    testBuffer5 = new int [bufferSize];
    SerialUSB.println(FreeRamMem());
    delete[] testBuffer5;
    SerialUSB.println(FreeRamMem());
}

void setup() {

    // Provisional For SCK
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    // BLINK LED
    pinMode(6, OUTPUT); //ROJO
    pinMode(12, OUTPUT); //VERDE
    pinMode(10, OUTPUT); //BLUE
    digitalWrite(6, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(10, HIGH);

    // Make the led blink
    digitalWrite(6, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(10, HIGH);
    delay(20);
    digitalWrite(6, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(10, HIGH);
    delay(20);

    // Do other stuff
    delay(2000);

    // Make the led blink
    digitalWrite(6, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(10, HIGH);
    delay(20);
    digitalWrite(6, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(10, HIGH);
    delay(20);

    SerialUSB.println(FreeRamMem());
}

void loop() {
    SerialUSB.println("Function");
    SerialUSB.println(FreeRamMem());
    allocateBuffer(1000);
    SerialUSB.println(FreeRamMem());
    SerialUSB.println("**");

    SerialUSB.println("ClassDynamic");
    TestClassDynamic *testDyn = new TestClassDynamic(1000);
    SerialUSB.println(FreeRamMem());
    testDyn->init();
    SerialUSB.println(FreeRamMem());
    testDyn->fill();
    delete testDyn;
    SerialUSB.println(FreeRamMem());
    SerialUSB.println("**");

    SerialUSB.println("ClassStatic");
    SerialUSB.println(FreeRamMem());
    TestClassStatic *testSt = new TestClassStatic(1000);
    SerialUSB.println(FreeRamMem());
    testSt->init();
    SerialUSB.println(FreeRamMem());
    testSt->fill();
    delete testSt;
    SerialUSB.println(FreeRamMem());
    SerialUSB.println("**");


    SerialUSB.println("Nested Class Dynamic");
    NoiseClass *noiseDyn = new NoiseClass(100);
    SerialUSB.println(FreeRamMem());
    noiseDyn->init();
    SerialUSB.println(FreeRamMem());
    SerialUSB.println(noiseDyn->getDyn());
    SerialUSB.println(FreeRamMem());
    noiseDyn->end();
    delete noiseDyn;
    SerialUSB.println(FreeRamMem());
    SerialUSB.println("**");

    SerialUSB.println("Nested Class Static");
    NoiseClass *noiseSt = new NoiseClass(100);
    SerialUSB.println(FreeRamMem());
    noiseSt->init();
    SerialUSB.println(FreeRamMem());
    SerialUSB.println(noiseSt->getSt());
    SerialUSB.println(FreeRamMem());
    noiseSt->end();
    delete noiseSt;
    SerialUSB.println(FreeRamMem());
    SerialUSB.println("**");



    A++;
    if (A==4){
        while (true) {

        }
    }

    // // Calloc case
    // SerialUSB.println("-------CALLOC------");
    // SerialUSB.print("Before Allocation FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // void* testBuffer = calloc(10, sizeof(int));    
    // SerialUSB.print("After calloc FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // free(testBuffer);
    // SerialUSB.print("After Free calloc FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // SerialUSB.println("-------MALLOC------");
    // SerialUSB.print("Before Allocation FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // void* testBuffer2 = malloc(10*sizeof(int));
    // SerialUSB.print("After malloc FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // free(testBuffer2);
    // SerialUSB.print("After Free malloc FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // SerialUSB.println("-------CASTING------");
    // SerialUSB.print("Before casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // int* testBuffer3 = (int*) calloc(1000, sizeof(int));  

    // for (int i=0;i<10;i++){
    //     testBuffer3 [i]=i*2;
    // }

    // SerialUSB.print("After casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // // for (int i=0;i<10;i++){
    // //     SerialUSB.println(testBuffer3 [i]);
    // // }

    // free(testBuffer3);
    // SerialUSB.print("After Free casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());


    // // for (int i=0;i<10;i++){
    // //     SerialUSB.println(testBuffer3 [i]);
    // // }

    // SerialUSB.print("After Free casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // SerialUSB.println("-------CALLOC CASTING------");
    // SerialUSB.print("Before casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // testBuffer4 = (int*) calloc(10, sizeof(int));  

    // for (int i=0;i<10;i++){
    //     testBuffer4 [i]=i;
    // }

    // for (int i=0;i<10;i++){
    //     SerialUSB.println(testBuffer4 [i]);
    // }

    // SerialUSB.print("After calloc casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // free(testBuffer4);
    // SerialUSB.print("After Free calloc casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // for (int i=0;i<10;i++){
    //     SerialUSB.println(testBuffer4 [i]);
    // }

    // SerialUSB.println("---------------------");

    // SerialUSB.println("-------NEW-DELETE------");
    // SerialUSB.print("Before casting FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());
    // // testBuffer5 = new int [1000];
    // int testBuffer5 [1000];

    // for (int i=0;i<1000;i++){
    //     testBuffer5 [i]=i;
    // }

    // // for (int i=0;i<1000;i++){
    // //     SerialUSB.println(testBuffer5 [i]);
    // // }

    // SerialUSB.print("After new FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // delete[] testBuffer5;
    // SerialUSB.print("After delete FreeRamMem");
    // SerialUSB.print("\t");
    // SerialUSB.println(FreeRamMem());

    // for (int i=0;i<10;i++){
    //     SerialUSB.println(testBuffer5 [i]);
    // }

    // SerialUSB.println("---------------------");
}

