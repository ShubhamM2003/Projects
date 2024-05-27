# Projects

Hey! I'd love to help you with that! Here's a Java program to find prime numbers from 1 to 100:

```java
public class PrimeNumbers {
    public static void main(String[] args) {
        for (int i = 1; i <= 100; i++) {
            if (isPrime(i)) {
                System.out.println(i);
            }
        }
    }

    public static boolean isPrime(int number) {
        if (number <= 1) {
            return false;
        }

        for (int i = 2; i <= Math.sqrt(number); i++) {
            if (number % i == 0) {
                return false;
            }
        }

        return true;
    }
}
```

This program will print all the prime numbers from 1 to 100. Let me know if you need any further help!