package cgl.iotrobots.collavoid.utils;


public class FootPrint {
    private double[] XArray;
    private double[] YArray;
    private double[] XArrayOriginal;
    private double[] YArrayOriginal;
    private int size;
    private final int step = 20;
    private boolean sorted=false;

    public FootPrint() {
        XArray = new double[step];
        YArray = new double[step];
        size = 0;
    }

    public FootPrint(int capacity) {
        XArray = new double[capacity];
        YArray = new double[capacity];
        this.size = 0;
    }

    public int size(){
        return size;
    }

    public void addPoint(double x, double y) {
        if (XArray.length <= this.size) {
            double[] Xtmp = new double[this.size + step];
            double[] Ytmp = new double[this.size + step];
            System.arraycopy(this.XArray, 0, Xtmp, 0, this.size);
            System.arraycopy(this.YArray, 0, Ytmp, 0, this.size);
            XArray = Xtmp;
            XArray[this.size] = x;
            YArray = Ytmp;
            YArray[this.size] = y;
        } else {
            XArray[this.size] = x;
            YArray[this.size] = y;
        }
        sorted=false;
        this.size += 1;
    }

    public double getX(int idx){
        return XArray[idx];
    }

    public double getY(int idx){
        return YArray[idx];
    }

    public double[] getXArray() {
        return XArray;
    }

    public double[] getYArray() {
        return YArray;
    }

    public void setPoints(double[] XArray,double[] YArray,int size) {
        if (this.size<size) {
            resizeArray(size);
        }
        System.arraycopy(XArray, 0, this.XArray, 0, size);
        System.arraycopy(YArray, 0, this.YArray, 0, size);
    }

    public void sort(){
        if (size>0) {
            resizeArrayOriginal(size);
            System.arraycopy(XArray, 0, XArrayOriginal, 0, size);
            System.arraycopy(YArray, 0, YArrayOriginal, 0, size);
            sortLexigraphic(this);
        }
            sorted=true;
    }

    public void unSort(){
        if (size>0&&sorted){
            System.arraycopy(XArrayOriginal, 0, XArray, 0, size);
            System.arraycopy(YArrayOriginal, 0, YArray, 0, size);
        }
        sorted=false;
    }

    private void resizeArray(int size){
        XArray=new double[size];
        YArray=new double[size];
        this.size=size;
        this.sorted=false;
    }

    private void resizeArrayOriginal(int size){
        XArrayOriginal=new double[size];
        YArrayOriginal=new double[size];
        this.size=size;
        this.sorted=false;
    }

    public static void sortLexigraphic(FootPrint fp){
        double[] maxminX=fp.getMinMaxX();
        double max=maxminX[1],min=maxminX[0];
        int size=fp.size();
        double delta=(max-min)/(size-1);
        int[][] bucket = new int[size][size]; // 用一个二维数组来表示桶

        for (int i = 0; i < size; i++) {
            bucket[i] = new int[size];
        }

        int[] count = new int[size];
        // initialize the size of each bucket to zero
        for (int i = 0; i < size; i++) {
            count[i] = 0;
        }

        for (int i = 0; i < size; i++) {
            double tmp = fp.getX(i);
            // get index according to the step
            int index = (int) ((tmp-min)/delta);
            // record index of the array at the end of the bucket
            bucket[index][count[index]] = i;
            int j = count[index]++;

            while (j > 0 && tmp <= fp.getX(bucket[index][j - 1])) // 对同一个桶内的元素进行插入排序
            {
                if (tmp==fp.getX(bucket[index][j - 1])){
                    if (fp.getY(i)>=fp.getY(bucket[index][j - 1]))
                        break;
                }
                bucket[index][j] = bucket[index][j - 1];
                j--;
            }
            bucket[index][j] = i;
        }

        double[] tmpX=new double[size];
        double[] tmpY=new double[size];
        int m = 0;
        for (int i = 0; i < size; i++) // 按序将桶内元素全部读出来
        {
            for (int j = 0; j < count[i]; j++) {
                tmpX[m] = fp.getX(bucket[i][j]);
                tmpY[m] = fp.getY(bucket[i][j]);
                m++;
            }
        }
        fp.setPoints(tmpX, tmpY, size);
    }

    private double[] getMinMaxX(){
        double min,max;
        double[] result=new double[2];
        //initialize max min
        if(XArray[0]<=XArray[1])
        {
            min = XArray[0];
            max = XArray[1];
        }
        else
        {
            min = XArray[1];
            max = XArray[0];
        }
        int i=0;
        if(size%2 != 0)
            i=1;
        while (i<size)
        {
            if(XArray[i]<=XArray[i+1])
            {
                if(XArray[i]<min)     min = XArray[i];
                if(XArray[i+1]>max) max = XArray[i+1];
            }
            else
            {
                if(XArray[i+1]<min) min = XArray[i+1];
                if(XArray[i]>max)     max =XArray[i];
            }
            i+=2;
        }
        result[0]=min;
        result[1]=max;
        return result;
    }

    public FootPrint minkowskiSum(final FootPrint polygon){
        FootPrint result=new FootPrint(size*polygon.size());
        for (int i = 0; i <  polygon.size(); i++) {
            for (int j = 0; j <  size; j++) {
                double x=polygon.getX(i)+XArray[j];
                double y=polygon.getY(i)+YArray[j];
                result.addPoint(x,y);
            }
        }
        return result;
    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    //Wikipedia Monotone chain...
    public FootPrint getConvexHullFootprint()
    {
        boolean sortLocally=false;
        int n = size, k = 0;
        double[] xarray=new double[2*n];
        double[] yarray=new double[2*n];
        FootPrint result=new FootPrint();

        // Sort points lexicographically
        if (!sorted) {
            this.sort();
            sortLocally=true;
        }

        //    ROS_WARN("points length %d", (int)P.size());

        // Build lower hull,
        for (int i = 0; i < n; i++) {
            while (k >= 2 && Vector2.det(xarray[k-2]-xarray[k-1],yarray[k-2]-yarray[k-1],
                    XArray[i]-xarray[k-1],YArray[i]-yarray[k-1])<= 0) k--;
            xarray[k]=XArray[i];
            yarray[k]=YArray[i];
            k++;
        }

        // Build upper hull
        for (int i = n-2, t = k+1; i >= 0; i--) {
            while (k >= t && Vector2.det(xarray[k-2]-xarray[k-1],yarray[k-2]-yarray[k-1],
                    XArray[i]-xarray[k-1],YArray[i]-yarray[k-1])<= 0) k--;
            xarray[k]=XArray[i];
            yarray[k]=YArray[i];
            k++;
        }

        result.setPoints(xarray,yarray,k);

        if (sortLocally)
            this.unSort();

        return result;
    }

}
