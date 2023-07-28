public class Test {

    public static void main(String[] args) {
        System.out.println("Hello world");
        System.out.print("{");
        int val = 1000;
        long sum = 0;
        int reverseIndex = 0;
        for (int i=0; i < 250; i++) {
            if (i > 0) System.out.print(",");
            if (i < 100) { //первые 100 шагов - замедление и реверс
                val =  900 + (int)(100* Math.abs(1/(Math.cos((double) i/100*Math.PI))));
                if (val > 5000 || val < 0) { //находим реверс
                    reverseIndex = i;
                    val = 5000;
                }
            }
            else {
                val =  1000 - (int)(250* Math.abs((Math.sin((double) (i-100)/150*Math.PI))));
            }

            System.out.print(val);
            sum += val;
        }
        System.out.println("};");
        System.out.println("Avg is " + (sum/250) + ", reverse at " + reverseIndex);
    }
}
